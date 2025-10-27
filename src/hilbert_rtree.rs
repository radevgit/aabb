//#![warn(unsafe_code)]

//! Hilbert R-tree implementation using unsafe memory layout for performance.
//!
//! All unsafe operations are internal implementation details. The public API is safe.
//! Memory is managed in a single buffer with type-punned box structures and indices.
//! Buffer invariants are maintained throughout the tree's lifetime.

use std::mem::size_of;

/// Box structure: minX, minY, maxX, maxY
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub(crate) struct Box {
    pub(crate) min_x: f64,
    pub(crate) min_y: f64,
    pub(crate) max_x: f64,
    pub(crate) max_y: f64,
}

impl Box {
    fn new(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Self {
        Box { min_x, min_y, max_x, max_y }
    }
}

/// Hilbert R-tree for spatial queries - following flatbush algorithm
///
/// Memory layout (in single buffer):
/// - Header: 8 bytes (magic, version, node_size, num_items)
/// - All boxes: num_total_nodes * 32 bytes (4 f64 per box)
/// - All indices: num_total_nodes * 4 bytes (u32 per node)
///
/// Leaf nodes occupy positions [0, num_items), parent nodes appended after.
/// Tree is built bottom-up with Hilbert curve ordering for spatial locality.
#[derive(Clone, Debug)]
pub struct HilbertRTree {
    /// Single buffer: header + boxes + indices
    data: Vec<u8>,
    /// Level boundaries: end position of each tree level
    pub(crate) level_bounds: Vec<usize>,
    /// Node size for tree construction
    pub(crate) node_size: usize,
    /// Number of leaf items
    pub(crate) num_items: usize,
    /// Current position during building
    pub(crate) position: usize,
    /// Bounding box of all items
    pub(crate) bounds: Box,
}

const MAX_HILBERT: u32 = u16::MAX as u32;
const DEFAULT_NODE_SIZE: usize = 16;
const HEADER_SIZE: usize = 8; // bytes

impl HilbertRTree {
    /// Creates a new empty Hilbert R-tree
    pub fn new() -> Self {
        HilbertRTree::with_capacity(0)
    }

    /// Creates a new Hilbert R-tree with preallocated capacity
    pub fn with_capacity(capacity: usize) -> Self {
        let data = if capacity > 0 {
            let mut v = Vec::new();
            v.resize(HEADER_SIZE + capacity * size_of::<Box>(), 0);
            v
        } else {
            Vec::new()
        };
        
        HilbertRTree {
            data,
            level_bounds: Vec::new(),
            node_size: DEFAULT_NODE_SIZE,
            num_items: 0,
            position: 0,
            bounds: Box::new(f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
        }
    }

    /// Adds a bounding box to the tree
    pub fn add(&mut self, min_x: f64, min_y: f64, max_x: f64, max_y: f64) {
        // Store boxes temporarily - will be reorganized during build
        if self.num_items == 0 {
            // Pre-allocate data buffer on first add - need space for header + boxes
            let capacity = 128; // Start with reasonable capacity
            self.data.resize(HEADER_SIZE + capacity * size_of::<Box>(), 0);
        }

        // Check if we need to expand
        let required_size = HEADER_SIZE + (self.num_items + 1) * size_of::<Box>();
        if required_size > self.data.len() {
            self.data.resize((self.data.len() * 2).max(required_size), 0);
        }

        let box_idx = HEADER_SIZE + self.num_items * size_of::<Box>();
        let box_ptr = &mut self.data[box_idx] as *mut u8 as *mut Box;
        unsafe {
            std::ptr::write_unaligned(box_ptr, Box::new(min_x, min_y, max_x, max_y));
        }

        self.bounds.min_x = self.bounds.min_x.min(min_x);
        self.bounds.min_y = self.bounds.min_y.min(min_y);
        self.bounds.max_x = self.bounds.max_x.max(max_x);
        self.bounds.max_y = self.bounds.max_y.max(max_y);

        self.num_items += 1;
    }

    /// Builds the Hilbert R-tree index
    pub fn build(&mut self) {
        if self.num_items == 0 {
            return;
        }

        let num_items = self.num_items;
        let node_size = self.node_size;

        // Calculate total nodes and level bounds
        let mut level_bounds = Vec::with_capacity(16); // Max tree depth ~16 for 1M items
        let mut count = num_items;
        let mut total_nodes = num_items;
        level_bounds.push(total_nodes);

        // Create parent levels until we have a single root
        loop {
            count = (count + node_size - 1) / node_size;
            total_nodes += count;
            level_bounds.push(total_nodes);
            if count <= 1 {
                break;
            }
        }

        // Resize data buffer to final size
        let data_size = HEADER_SIZE + total_nodes * (size_of::<Box>() + size_of::<u32>());
        self.data.resize(data_size, 0);

        // Write header
        self.data[0] = 0xfb; // magic
        self.data[1] = 0x20; // version 2 + double type (8)
        self.data[2..4].copy_from_slice(&(node_size as u16).to_le_bytes());
        self.data[4..8].copy_from_slice(&(num_items as u32).to_le_bytes());

        self.level_bounds = level_bounds;
        self.position = 0;

        // If all items fit in one node, create a root level
        if num_items <= node_size {
            // Initialize all leaf indices first
            let indices_start = HEADER_SIZE + total_nodes * size_of::<Box>();
            for i in 0..num_items {
                let idx_ptr = &mut self.data[indices_start + i * size_of::<u32>()] as *mut u8 as *mut u32;
                unsafe {
                    std::ptr::write_unaligned(idx_ptr, i as u32);
                }
            }
            
            // Write the root node box at position num_items
            let root_idx = HEADER_SIZE + num_items * size_of::<Box>();
            let root_ptr = &mut self.data[root_idx] as *mut u8 as *mut Box;
            unsafe {
                std::ptr::write_unaligned(root_ptr, self.bounds);
            }
            
            // Write the root node index (pointer to first child at position 0)
            let root_idx_ptr = &mut self.data[indices_start + num_items * size_of::<u32>()] as *mut u8 as *mut u32;
            unsafe {
                std::ptr::write_unaligned(root_idx_ptr, 0u32 << 2u32); // First child at position 0
            }
            return;
        }

        // Compute Hilbert values for leaves
        let hilbert_width = MAX_HILBERT as f64 / (self.bounds.max_x - self.bounds.min_x);
        let hilbert_height = MAX_HILBERT as f64 / (self.bounds.max_y - self.bounds.min_y);

        let mut hilbert_values = vec![0u32; num_items];
        for i in 0..num_items {
            let box_data = self.get_box(i);
            let center_x = ((box_data.min_x + box_data.max_x) / 2.0 - self.bounds.min_x) * hilbert_width;
            let center_y = ((box_data.min_y + box_data.max_y) / 2.0 - self.bounds.min_y) * hilbert_height;
            let hx = center_x.max(0.0).min(MAX_HILBERT as f64 - 1.0) as u32;
            let hy = center_y.max(0.0).min(MAX_HILBERT as f64 - 1.0) as u32;
            hilbert_values[i] = hilbert_xy_to_index(hx, hy);
        }

        // Initialize leaf indices BEFORE sorting
        let indices_start = HEADER_SIZE + total_nodes * size_of::<Box>();
        for i in 0..num_items {
            let idx_ptr = &mut self.data[indices_start + i * size_of::<u32>()] as *mut u8 as *mut u32;
            unsafe {
                std::ptr::write_unaligned(idx_ptr, i as u32);
            }
        }

        // Sort leaves by Hilbert value
        self.quicksort(&mut hilbert_values, 0, num_items - 1);

        // Build parent levels
        let mut pos = 0usize;
        for level_idx in 0..self.level_bounds.len() - 1 {
            let level_end = self.level_bounds[level_idx];
            let mut parent_pos = level_end;

            while pos < level_end {
                let node_index = (pos as u32) << 2u32; // for JS compatibility
                let mut node_box = self.get_box(pos);

                // Merge up to node_size children
                for _ in 0..node_size {
                    if pos >= level_end {
                        break;
                    }
                    let child_box = self.get_box(pos);
                    node_box.min_x = node_box.min_x.min(child_box.min_x);
                    node_box.min_y = node_box.min_y.min(child_box.min_y);
                    node_box.max_x = node_box.max_x.max(child_box.max_x);
                    node_box.max_y = node_box.max_y.max(child_box.max_y);
                    pos += 1;
                }

                // Write parent node box
                let box_idx = HEADER_SIZE + parent_pos * size_of::<Box>();
                let box_ptr = &mut self.data[box_idx] as *mut u8 as *mut Box;
                unsafe {
                    std::ptr::write_unaligned(box_ptr, node_box);
                }

                // Write parent node index
                let idx_ptr = &mut self.data[indices_start + parent_pos * size_of::<u32>()] as *mut u8 as *mut u32;
                unsafe {
                    std::ptr::write_unaligned(idx_ptr, node_index);
                }

                parent_pos += 1;
            }
            pos = level_end;
        }
    }

    /// Returns the number of items
    pub fn len(&self) -> usize {
        self.num_items
    }

    /// Returns whether the tree is empty
    pub fn is_empty(&self) -> bool {
        self.num_items == 0
    }

    /// Internal: Generic tree traversal for queries

    /// Query for boxes intersecting the given rectangle
    pub fn query_intersecting(
        &self,
        min_x: f64,
        min_y: f64,
        max_x: f64,
        max_y: f64,
        results: &mut Vec<usize>,
    ) {
        if self.num_items == 0 || self.level_bounds.is_empty() {
            return;
        }

        results.clear();
        
        let mut queue = Vec::new();
        let total_nodes = self.level_bounds.last().copied().unwrap_or(0);
        let mut node_index = total_nodes - 1;
        
        loop {
            // Find the end index of the node (upper bound)
            let node_end = self.upper_bound(node_index);
            let end_pos = (node_index + self.node_size).min(node_end);
            
            // Search through child nodes
            for pos in node_index..end_pos {
                let node_box = self.get_box(pos);
                
                // Check if node bbox intersects with query bbox
                if max_x < node_box.min_x || max_y < node_box.min_y ||
                   min_x > node_box.max_x || min_y > node_box.max_y {
                    continue;
                }
                
                let index = self.get_index(pos);
                if node_index >= self.num_items {
                    // This is a parent node; add to queue
                    queue.push((index >> 2) as usize);
                } else {
                    // This is a leaf item
                    results.push(index as usize);
                }
            }
            
            if queue.is_empty() {
                break;
            }
            
            node_index = queue.remove(0);
        }
    }

    /// Query for K nearest neighbor boxes to a point
    pub fn query_nearest_k(
        &self,
        point_x: f64,
        point_y: f64,
        k: usize,
        results: &mut Vec<usize>,
    ) {
        if self.num_items == 0 || self.level_bounds.is_empty() {
            results.clear();
            return;
        }

        if k == 0 {
            results.clear();
            return;
        }

        // Collect all leaf nodes with their distances
        let mut candidates: Vec<(u64, usize)> = Vec::with_capacity(self.num_items);

        // Process all leaf nodes (first level_bounds[0] positions)
        let num_leaves = self.level_bounds[0];
        for pos in 0..num_leaves {
            let node_box = self.get_box(pos);
            let dx = self.axis_distance(point_x, node_box.min_x, node_box.max_x);
            let dy = self.axis_distance(point_y, node_box.min_y, node_box.max_y);
            let dist_sq = dx * dx + dy * dy;
            let dist_bits = dist_sq.to_bits();
            let index = self.get_index(pos) as usize;
            candidates.push((dist_bits, index));
        }

        // Sort by distance and keep only K
        candidates.sort_by(|a, b| a.0.cmp(&b.0));
        candidates.truncate(k);

        results.clear();
        for (_, idx) in candidates {
            results.push(idx);
        }
    }

    /// Query for boxes that contain a specific point
    pub fn query_point(&self, x: f64, y: f64, results: &mut Vec<usize>) {
        if self.num_items == 0 || self.level_bounds.is_empty() {
            return;
        }

        results.clear();
        
        let mut queue = Vec::new();
        let total_nodes = self.level_bounds.last().copied().unwrap_or(0);
        let mut node_index = total_nodes - 1;
        
        loop {
            let node_end = self.upper_bound(node_index);
            let end_pos = (node_index + self.node_size).min(node_end);
            
            for pos in node_index..end_pos {
                let node_box = self.get_box(pos);
                
                // Check if point is inside box
                if x < node_box.min_x || x > node_box.max_x ||
                   y < node_box.min_y || y > node_box.max_y {
                    continue;
                }
                
                let index = self.get_index(pos);
                if node_index >= self.num_items {
                    queue.push((index >> 2) as usize);
                } else {
                    results.push(index as usize);
                }
            }
            
            if queue.is_empty() {
                break;
            }
            
            node_index = queue.remove(0);
        }
    }

    /// Query for boxes that completely contain a given rectangle
    pub fn query_containing(&self, min_x: f64, min_y: f64, max_x: f64, max_y: f64, results: &mut Vec<usize>) {
        if self.num_items == 0 || self.level_bounds.is_empty() {
            return;
        }

        results.clear();
        
        let mut queue = Vec::new();
        let total_nodes = self.level_bounds.last().copied().unwrap_or(0);
        let mut node_index = total_nodes - 1;
        
        loop {
            let node_end = self.upper_bound(node_index);
            let end_pos = (node_index + self.node_size).min(node_end);
            
            for pos in node_index..end_pos {
                let node_box = self.get_box(pos);
                
                // Check if node contains the query rectangle
                if node_box.min_x <= min_x && node_box.max_x >= max_x &&
                   node_box.min_y <= min_y && node_box.max_y >= max_y {
                    
                    let index = self.get_index(pos);
                    if node_index >= self.num_items {
                        queue.push((index >> 2) as usize);
                    } else {
                        results.push(index as usize);
                    }
                }
            }
            
            if queue.is_empty() {
                break;
            }
            
            node_index = queue.remove(0);
        }
    }

    /// Query for boxes that are contained within a given rectangle
    pub fn query_contained_by(&self, min_x: f64, min_y: f64, max_x: f64, max_y: f64, results: &mut Vec<usize>) {
        if self.num_items == 0 || self.level_bounds.is_empty() {
            return;
        }

        results.clear();
        
        let mut queue = Vec::new();
        let total_nodes = self.level_bounds.last().copied().unwrap_or(0);
        let mut node_index = total_nodes - 1;
        
        loop {
            let node_end = self.upper_bound(node_index);
            let end_pos = (node_index + self.node_size).min(node_end);
            
            for pos in node_index..end_pos {
                let node_box = self.get_box(pos);
                
                if node_index >= self.num_items {
                    // This is a parent node - check if it could have matching children
                    // (any overlap with query region)
                    if node_box.max_x >= min_x && node_box.max_y >= min_y &&
                       node_box.min_x <= max_x && node_box.min_y <= max_y {
                        let index = self.get_index(pos);
                        queue.push((index >> 2) as usize);
                    }
                } else {
                    // This is a leaf - check if fully contained
                    if node_box.min_x >= min_x && node_box.max_x <= max_x &&
                       node_box.min_y >= min_y && node_box.max_y <= max_y {
                        let index = self.get_index(pos);
                        results.push(index as usize);
                    }
                }
            }
            
            if queue.is_empty() {
                break;
            }
            
            node_index = queue.remove(0);
        }
    }

    /// Query for the single nearest box to a point
    pub fn query_nearest(&self, x: f64, y: f64) -> Option<usize> {
        if self.num_items == 0 || self.level_bounds.is_empty() {
            return None;
        }

        let mut min_dist_bits = u64::MAX;
        let mut result = None;

        // Check all leaf nodes
        let num_leaves = self.level_bounds[0];
        for pos in 0..num_leaves {
            let node_box = self.get_box(pos);
            let dx = self.axis_distance(x, node_box.min_x, node_box.max_x);
            let dy = self.axis_distance(y, node_box.min_y, node_box.max_y);
            let dist_sq = dx * dx + dy * dy;
            let dist_bits = dist_sq.to_bits();

            if dist_bits < min_dist_bits {
                min_dist_bits = dist_bits;
                result = Some(self.get_index(pos) as usize);
            }
        }

        result
    }

    /// Query for first K intersecting boxes (stops after K results)
    pub fn query_intersecting_k(
        &self,
        min_x: f64,
        min_y: f64,
        max_x: f64,
        max_y: f64,
        k: usize,
        results: &mut Vec<usize>,
    ) {
        if self.num_items == 0 || self.level_bounds.is_empty() || k == 0 {
            results.clear();
            return;
        }

        results.clear();
        
        let mut queue = Vec::with_capacity(self.level_bounds.len() * 2);
        let total_nodes = self.level_bounds.last().copied().unwrap_or(0);
        let mut node_index = total_nodes - 1;
        
        loop {
            if results.len() >= k {
                break;
            }

            let node_end = self.upper_bound(node_index);
            let end_pos = (node_index + self.node_size).min(node_end);
            
            for pos in node_index..end_pos {
                if results.len() >= k {
                    break;
                }

                let node_box = self.get_box(pos);
                
                if max_x < node_box.min_x || max_y < node_box.min_y ||
                   min_x > node_box.max_x || min_y > node_box.max_y {
                    continue;
                }
                
                let index = self.get_index(pos);
                if pos < self.num_items {
                    results.push(index as usize);
                } else {
                    queue.push((index >> 2) as usize);
                }
            }
            
            if queue.is_empty() {
                break;
            }
            
            node_index = queue.remove(0);
        }
    }

    /// Query for boxes within a distance of a point
    pub fn query_within_distance(&self, x: f64, y: f64, max_distance: f64, results: &mut Vec<usize>) {
        if self.num_items == 0 || self.level_bounds.is_empty() || max_distance < 0.0 {
            results.clear();
            return;
        }

        results.clear();
        
        let max_dist_sq = max_distance * max_distance;
        let mut queue = Vec::new();
        let total_nodes = self.level_bounds.last().copied().unwrap_or(0);
        let mut node_index = total_nodes - 1;
        
        loop {
            let node_end = self.upper_bound(node_index);
            let end_pos = (node_index + self.node_size).min(node_end);
            
            for pos in node_index..end_pos {
                let node_box = self.get_box(pos);
                let dx = self.axis_distance(x, node_box.min_x, node_box.max_x);
                let dy = self.axis_distance(y, node_box.min_y, node_box.max_y);
                let dist_sq = dx * dx + dy * dy;

                if dist_sq <= max_dist_sq {
                    let index = self.get_index(pos);
                    if node_index >= self.num_items {
                        queue.push((index >> 2) as usize);
                    } else {
                        results.push(index as usize);
                    }
                }
            }
            
            if queue.is_empty() {
                break;
            }
            
            node_index = queue.remove(0);
        }
    }

    /// Query for boxes intersecting a circular region
    pub fn query_circle(&self, center_x: f64, center_y: f64, radius: f64, results: &mut Vec<usize>) {
        if self.num_items == 0 || self.level_bounds.is_empty() || radius < 0.0 {
            results.clear();
            return;
        }

        results.clear();
        
        let radius_sq = radius * radius;
        let mut queue = Vec::new();
        let total_nodes = self.level_bounds.last().copied().unwrap_or(0);
        let mut node_index = total_nodes - 1;
        
        loop {
            let node_end = self.upper_bound(node_index);
            let end_pos = (node_index + self.node_size).min(node_end);
            
            for pos in node_index..end_pos {
                let node_box = self.get_box(pos);
                // Distance from circle center to box
                let dx = self.axis_distance(center_x, node_box.min_x, node_box.max_x);
                let dy = self.axis_distance(center_y, node_box.min_y, node_box.max_y);
                let dist_sq = dx * dx + dy * dy;

                if dist_sq <= radius_sq {
                    let index = self.get_index(pos);
                    if node_index >= self.num_items {
                        queue.push((index >> 2) as usize);
                    } else {
                        results.push(index as usize);
                    }
                }
            }
            
            if queue.is_empty() {
                break;
            }
            
            node_index = queue.remove(0);
        }
    }

    /// Query for boxes in a directional path (rectangle moving in direction)
    pub fn query_in_direction(
        &self,
        min_x: f64,
        min_y: f64,
        max_x: f64,
        max_y: f64,
        dir_x: f64,
        dir_y: f64,
        distance: f64,
        results: &mut Vec<usize>,
    ) {
        if self.num_items == 0 || self.level_bounds.is_empty() || distance < 0.0 {
            results.clear();
            return;
        }

        results.clear();
        
        // Normalize direction vector
        let dir_len_sq = dir_x * dir_x + dir_y * dir_y;
        if dir_len_sq <= 0.0 {
            return;
        }
        let dir_len = dir_len_sq.sqrt();
        let norm_dir_x = dir_x / dir_len;
        let norm_dir_y = dir_y / dir_len;
        
        // Calculate movement vector
        let dx = norm_dir_x * distance;
        let dy = norm_dir_y * distance;
        let sweep_min_x = min_x.min(min_x + dx);
        let sweep_min_y = min_y.min(min_y + dy);
        let sweep_max_x = max_x.max(max_x + dx);
        let sweep_max_y = max_y.max(max_y + dy);

        let mut queue = Vec::new();
        let total_nodes = self.level_bounds.last().copied().unwrap_or(0);
        let mut node_index = total_nodes - 1;
        
        loop {
            let node_end = self.upper_bound(node_index);
            let end_pos = (node_index + self.node_size).min(node_end);
            
            for pos in node_index..end_pos {
                let node_box = self.get_box(pos);
                
                // Check if box intersects the sweep area (AABB intersection)
                if sweep_max_x < node_box.min_x || sweep_max_y < node_box.min_y ||
                   sweep_min_x > node_box.max_x || sweep_min_y > node_box.max_y {
                    continue;
                }
                
                let index = self.get_index(pos);
                if node_index >= self.num_items {
                    queue.push((index >> 2) as usize);
                } else {
                    results.push(index as usize);
                }
            }
            
            if queue.is_empty() {
                break;
            }
            
            node_index = queue.remove(0);
        }
    }

    /// Query for K nearest boxes in a directional path
    pub fn query_in_direction_k(
        &self,
        min_x: f64,
        min_y: f64,
        max_x: f64,
        max_y: f64,
        dir_x: f64,
        dir_y: f64,
        k: usize,
        distance: f64,
        results: &mut Vec<usize>,
    ) {
        if self.num_items == 0 || self.level_bounds.is_empty() || distance < 0.0 || k == 0 {
            results.clear();
            return;
        }

        // Normalize direction vector
        let dir_len_sq = dir_x * dir_x + dir_y * dir_y;
        if dir_len_sq <= 0.0 {
            results.clear();
            return;
        }
        let dir_len = dir_len_sq.sqrt();
        let norm_dir_x = dir_x / dir_len;
        let norm_dir_y = dir_y / dir_len;
        
        // Calculate movement vector
        let dx = norm_dir_x * distance;
        let dy = norm_dir_y * distance;
        let sweep_min_x = min_x.min(min_x + dx);
        let sweep_min_y = min_y.min(min_y + dy);
        let sweep_max_x = max_x.max(max_x + dx);
        let sweep_max_y = max_y.max(max_y + dy);

        let mut candidates: Vec<(f64, usize)> = Vec::new();

        let mut queue = Vec::with_capacity(self.level_bounds.len() * 2);
        let total_nodes = self.level_bounds.last().copied().unwrap_or(0);
        queue.push(total_nodes - 1);

        while let Some(node_idx) = queue.pop() {
            let node_end = self.upper_bound(node_idx);
            let end_pos = (node_idx + self.node_size).min(node_end);

            for pos in node_idx..end_pos {
                let node_box = self.get_box(pos);
                
                // Check if box intersects the sweep area (AABB intersection)
                if sweep_max_x < node_box.min_x || sweep_max_y < node_box.min_y ||
                   sweep_min_x > node_box.max_x || sweep_min_y > node_box.max_y {
                    continue;
                }

                let index = self.get_index(pos) as usize;

                if pos < self.num_items {
                    // Calculate distance to box center along direction
                    let box_center_x = (node_box.min_x + node_box.max_x) / 2.0;
                    let box_center_y = (node_box.min_y + node_box.max_y) / 2.0;
                    
                    let relative_x = box_center_x - min_x;
                    let relative_y = box_center_y - min_y;
                    
                    // Distance along direction (use normalized direction)
                    let dist_along_dir = relative_x * norm_dir_x + relative_y * norm_dir_y;
                    candidates.push((dist_along_dir, index));
                } else {
                    queue.push(index >> 2);
                }
            }
        }

        // Sort by distance along direction and take first K
        candidates.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        
        results.clear();
        for (_, idx) in candidates.iter().take(k) {
            results.push(*idx);
        }
    }

    // --- Private helpers ---

    /// Get box at position (using slice from_raw_parts - Option 4)
    #[inline]
    pub(crate) fn get_box(&self, pos: usize) -> Box {
        let idx = HEADER_SIZE + pos * size_of::<Box>();
        let box_slice = unsafe {
            std::slice::from_raw_parts(&self.data[idx] as *const u8 as *const Box, 1)
        };
        box_slice[0]
    }

    /// Get index at position (using slice from_raw_parts - Option 4)
    #[inline]
    pub(crate) fn get_index(&self, pos: usize) -> u32 {
        let total_nodes = self.level_bounds.last().copied().unwrap_or(0);
        let indices_start = HEADER_SIZE + total_nodes * size_of::<Box>();
        let idx_slice = unsafe {
            std::slice::from_raw_parts(&self.data[indices_start + pos * size_of::<u32>()] as *const u8 as *const u32, 1)
        };
        idx_slice[0]
    }

    /// Get distance along an axis
    #[inline]
    fn axis_distance(&self, coordinate: f64, min: f64, max: f64) -> f64 {
        if coordinate < min {
            min - coordinate
        } else if coordinate > max {
            coordinate - max
        } else {
            0.0
        }
    }

    /// Find upper bound of a node in level_bounds
    #[inline]
    fn upper_bound(&self, node_index: usize) -> usize {
        for &bound in &self.level_bounds {
            if bound > node_index {
                return bound;
            }
        }
        self.level_bounds.last().copied().unwrap_or(0)
    }

    /// Quicksort by Hilbert value, also reordering boxes and indices
    fn quicksort(&mut self, hilbert_values: &mut [u32], left: usize, right: usize) {
        if left >= right {
            return;
        }

        let pivot = self.median_of_three(hilbert_values, left, right);
        let mut pivot_left = left as i32 - 1;
        let mut pivot_right = right as i32 + 1;

        loop {
            loop {
                pivot_left += 1;
                if hilbert_values[pivot_left as usize] >= pivot {
                    break;
                }
            }
            loop {
                pivot_right -= 1;
                if hilbert_values[pivot_right as usize] <= pivot {
                    break;
                }
            }

            if pivot_left >= pivot_right {
                break;
            }

            self.swap_elements(hilbert_values, pivot_left as usize, pivot_right as usize);
        }

        if pivot_right as usize > left {
            self.quicksort(hilbert_values, left, pivot_right as usize);
        }
        if (pivot_right as usize + 1) < right {
            self.quicksort(hilbert_values, pivot_right as usize + 1, right);
        }
    }

    /// Median of three for quicksort pivot selection
    fn median_of_three(&self, values: &[u32], left: usize, right: usize) -> u32 {
        let mid = (left + right) / 2;
        let a = values[left];
        let b = values[mid];
        let c = values[right];

        let x = a.max(b);
        if c > x {
            x
        } else if x == a {
            b.max(c)
        } else if x == b {
            a.max(c)
        } else {
            c
        }
    }

    /// Swap two elements (including boxes and indices)
    fn swap_elements(&mut self, hilbert_values: &mut [u32], left: usize, right: usize) {
        hilbert_values.swap(left, right);

        // Swap boxes
        let left_box_idx = HEADER_SIZE + left * size_of::<Box>();
        let right_box_idx = HEADER_SIZE + right * size_of::<Box>();
        
        let left_box = self.get_box(left);
        let right_box = self.get_box(right);
        
        let left_ptr = &mut self.data[left_box_idx] as *mut u8 as *mut Box;
        let right_ptr = &mut self.data[right_box_idx] as *mut u8 as *mut Box;
        
        unsafe {
            std::ptr::write_unaligned(left_ptr, right_box);
            std::ptr::write_unaligned(right_ptr, left_box);
        }

        // Swap indices
        let total_nodes = self.level_bounds.last().copied().unwrap_or(0);
        let indices_start = HEADER_SIZE + total_nodes * size_of::<Box>();
        
        let left_idx_ptr = &mut self.data[indices_start + left * size_of::<u32>()] as *mut u8 as *mut u32;
        let right_idx_ptr = &mut self.data[indices_start + right * size_of::<u32>()] as *mut u8 as *mut u32;
        
        unsafe {
            let left_idx = std::ptr::read_unaligned(left_idx_ptr);
            let right_idx = std::ptr::read_unaligned(right_idx_ptr);
            std::ptr::write_unaligned(left_idx_ptr, right_idx);
            std::ptr::write_unaligned(right_idx_ptr, left_idx);
        }
    }
}

impl Default for HilbertRTree {
    fn default() -> Self {
        Self::new()
    }
}

/// Hilbert curve index computation
/// From https://github.com/rawrunprotected/hilbert_curves (public domain)
fn interleave(mut x: u32) -> u32 {
    x = (x | (x << 8)) & 0x00FF00FF;
    x = (x | (x << 4)) & 0x0F0F0F0F;
    x = (x | (x << 2)) & 0x33333333;
    x = (x | (x << 1)) & 0x55555555;
    x
}

#[allow(non_snake_case)]
fn hilbert_xy_to_index(x: u32, y: u32) -> u32 {
    // Initial prefix scan round, prime with x and y
    let mut a = x ^ y;
    let mut b = 0xFFFF ^ a;
    let mut c = 0xFFFF ^ (x | y);
    let mut d = x & (y ^ 0xFFFF);
    let mut A = a | (b >> 1);
    let mut B = (a >> 1) ^ a;
    let mut C = ((c >> 1) ^ (b & (d >> 1))) ^ c;
    let mut D = ((a & (c >> 1)) ^ (d >> 1)) ^ d;

    a = A;
    b = B;
    c = C;
    d = D;
    A = (a & (a >> 2)) ^ (b & (b >> 2));
    B = (a & (b >> 2)) ^ (b & ((a ^ b) >> 2));
    C ^= (a & (c >> 2)) ^ (b & (d >> 2));
    D ^= (b & (c >> 2)) ^ ((a ^ b) & (d >> 2));

    a = A;
    b = B;
    c = C;
    d = D;
    A = (a & (a >> 4)) ^ (b & (b >> 4));
    B = (a & (b >> 4)) ^ (b & ((a ^ b) >> 4));
    C ^= (a & (c >> 4)) ^ (b & (d >> 4));
    D ^= (b & (c >> 4)) ^ ((a ^ b) & (d >> 4));

    // Final round and projection
    a = A;
    b = B;
    c = C;
    d = D;
    C ^= (a & (c >> 8)) ^ (b & (d >> 8));
    D ^= (b & (c >> 8)) ^ ((a ^ b) & (d >> 8));

    // Undo transformation prefix scan
    a = C ^ (C >> 1);
    b = D ^ (D >> 1);

    // Recover index bits
    let i0 = x ^ y;
    let i1 = b | (0xFFFF ^ (i0 | a));

    (interleave(i1) << 1) | interleave(i0)
}
