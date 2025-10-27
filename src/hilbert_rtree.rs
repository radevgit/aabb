#![warn(unsafe_code)]

//! Hilbert R-tree implementation using unsafe memory layout for performance.
//!
//! All unsafe operations are internal implementation details. The public API is safe.
//! Memory is managed in a single buffer with type-punned box structures and indices.
//! Buffer invariants are maintained throughout the tree's lifetime.

use std::mem;

/// Box structure: minX, minY, maxX, maxY
#[repr(C)]
#[derive(Clone, Copy, Debug)]
struct Box {
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
}

impl Box {
    fn new(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Self {
        Box { min_x, min_y, max_x, max_y }
    }
}

/// Hilbert R-tree for spatial queries - following flatbush algorithm (C++ version)
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
    level_bounds: Vec<usize>,
    /// Node size for tree construction
    node_size: usize,
    /// Number of leaf items
    num_items: usize,
    /// Current position during building
    position: usize,
    /// Bounding box of all items
    bounds: Box,
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
        HilbertRTree {
            data: Vec::new(),
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
            self.data.resize(HEADER_SIZE + capacity * mem::size_of::<Box>(), 0);
        }

        // Check if we need to expand
        let required_size = HEADER_SIZE + (self.num_items + 1) * mem::size_of::<Box>();
        if required_size > self.data.len() {
            self.data.resize((self.data.len() * 2).max(required_size), 0);
        }

        let box_idx = HEADER_SIZE + self.num_items * mem::size_of::<Box>();
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
        let mut level_bounds = Vec::new();
        let mut count = num_items;
        let mut total_nodes = num_items;
        level_bounds.push(total_nodes);

        loop {
            count = (count + node_size - 1) / node_size;
            total_nodes += count;
            level_bounds.push(total_nodes);
            if count <= 1 {
                break;
            }
        }

        // Resize data buffer to final size
        let data_size = HEADER_SIZE + total_nodes * (mem::size_of::<Box>() + mem::size_of::<u32>());
        self.data.resize(data_size, 0);

        // Write header
        self.data[0] = 0xfb; // magic
        self.data[1] = 0x30; // version 3 + double type (8)
        self.data[2..4].copy_from_slice(&(node_size as u16).to_le_bytes());
        self.data[4..8].copy_from_slice(&(num_items as u32).to_le_bytes());

        self.level_bounds = level_bounds;
        self.position = 0;

        // If all items fit in one node, just create root bounds
        if num_items <= node_size {
            let root_idx = HEADER_SIZE + num_items * mem::size_of::<Box>();
            let root_ptr = &mut self.data[root_idx] as *mut u8 as *mut Box;
            unsafe {
                std::ptr::write_unaligned(root_ptr, self.bounds);
            }
            
            let indices_start = HEADER_SIZE + total_nodes * mem::size_of::<Box>();
            let indices_ptr = &mut self.data[indices_start + num_items * mem::size_of::<u32>()] as *mut u8 as *mut u32;
            unsafe {
                std::ptr::write_unaligned(indices_ptr, 0);
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

        // Sort leaves by Hilbert value
        self.quicksort(&mut hilbert_values, 0, num_items - 1);

        // Initialize leaf indices
        let indices_start = HEADER_SIZE + total_nodes * mem::size_of::<Box>();
        for i in 0..num_items {
            let idx_ptr = &mut self.data[indices_start + i * mem::size_of::<u32>()] as *mut u8 as *mut u32;
            unsafe {
                std::ptr::write_unaligned(idx_ptr, i as u32);
            }
        }

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
                let box_idx = HEADER_SIZE + parent_pos * mem::size_of::<Box>();
                let box_ptr = &mut self.data[box_idx] as *mut u8 as *mut Box;
                unsafe {
                    std::ptr::write_unaligned(box_ptr, node_box);
                }

                // Write parent node index
                let idx_ptr = &mut self.data[indices_start + parent_pos * mem::size_of::<u32>()] as *mut u8 as *mut u32;
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
            return;
        }

        results.clear();
        
        let mut queue: Vec<(u64, usize)> = Vec::new(); // (dist_bits, index)
        let mut node_index = self.level_bounds.last().copied().unwrap_or(0) - 1;
        
        loop {
            let node_end = self.upper_bound(node_index);
            let end_pos = (node_index + self.node_size).min(node_end);
            
            for pos in node_index..end_pos {
                let node_box = self.get_box(pos);
                let dx = self.axis_distance(point_x, node_box.min_x, node_box.max_x);
                let dy = self.axis_distance(point_y, node_box.min_y, node_box.max_y);
                let dist_sq = dx * dx + dy * dy;
                let dist_bits = dist_sq.to_bits();
                
                let index = self.get_index(pos) as usize;
                if pos >= self.num_items {
                    queue.push((dist_bits, index >> 2));
                } else {
                    queue.push((dist_bits, (index << 1) + 1));
                }
            }
            
            if queue.is_empty() {
                break;
            }
            
            queue.sort_by(|a, b| a.0.cmp(&b.0));
            let (_, idx) = queue.remove(0);
            node_index = idx;
            
            if results.len() >= k {
                break;
            }
        }
        
        // Collect results
        for (_, idx) in queue {
            if idx & 1 == 1 {
                results.push(idx >> 1);
                if results.len() >= k {
                    break;
                }
            }
        }
    }

    // --- Private helpers ---

    /// Get box at position (using slice from_raw_parts - Option 4)
    #[inline]
    fn get_box(&self, pos: usize) -> Box {
        let idx = HEADER_SIZE + pos * mem::size_of::<Box>();
        let box_slice = unsafe {
            std::slice::from_raw_parts(&self.data[idx] as *const u8 as *const Box, 1)
        };
        box_slice[0]
    }

    /// Get index at position (using slice from_raw_parts - Option 4)
    #[inline]
    fn get_index(&self, pos: usize) -> u32 {
        let total_nodes = self.level_bounds.last().copied().unwrap_or(0);
        let indices_start = HEADER_SIZE + total_nodes * mem::size_of::<Box>();
        let idx_slice = unsafe {
            std::slice::from_raw_parts(&self.data[indices_start + pos * mem::size_of::<u32>()] as *const u8 as *const u32, 1)
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
        let left_box_idx = HEADER_SIZE + left * mem::size_of::<Box>();
        let right_box_idx = HEADER_SIZE + right * mem::size_of::<Box>();
        
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
        let indices_start = HEADER_SIZE + total_nodes * mem::size_of::<Box>();
        
        let left_idx_ptr = &mut self.data[indices_start + left * mem::size_of::<u32>()] as *mut u8 as *mut u32;
        let right_idx_ptr = &mut self.data[indices_start + right * mem::size_of::<u32>()] as *mut u8 as *mut u32;
        
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
fn hilbert_xy_to_index(mut x: u32, mut y: u32) -> u32 {
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
    A = ((a & (a >> 2)) ^ (b & (b >> 2)));
    B = ((a & (b >> 2)) ^ (b & ((a ^ b) >> 2)));
    C ^= ((a & (c >> 2)) ^ (b & (d >> 2)));
    D ^= ((b & (c >> 2)) ^ ((a ^ b) & (d >> 2)));

    a = A;
    b = B;
    c = C;
    d = D;
    A = ((a & (a >> 4)) ^ (b & (b >> 4)));
    B = ((a & (b >> 4)) ^ (b & ((a ^ b) >> 4)));
    C ^= ((a & (c >> 4)) ^ (b & (d >> 4)));
    D ^= ((b & (c >> 4)) ^ ((a ^ b) & (d >> 4)));

    // Final round and projection
    a = A;
    b = B;
    c = C;
    d = D;
    C ^= ((a & (c >> 8)) ^ (b & (d >> 8)));
    D ^= ((b & (c >> 8)) ^ ((a ^ b) & (d >> 8)));

    // Undo transformation prefix scan
    a = C ^ (C >> 1);
    b = D ^ (D >> 1);

    // Recover index bits
    let i0 = x ^ y;
    let i1 = b | (0xFFFF ^ (i0 | a));

    ((interleave(i1) << 1) | interleave(i0))
}
