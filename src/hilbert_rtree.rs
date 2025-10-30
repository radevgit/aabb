//#![warn(unsafe_code)]

//! Hilbert R-tree implementation using unsafe memory layout for performance.
//!
//! All unsafe operations are internal implementation details. The public API is safe.
//! Memory is managed in a single buffer with type-punned box structures and indices.
//! Buffer invariants are maintained throughout the tree's lifetime.

use std::mem::size_of;
use std::collections::VecDeque;

/// Box structure: minX, minY, maxX, maxY
#[derive(Clone, Copy, Debug)]
pub(crate) struct Box {
    pub(crate) min_x: f64,
    pub(crate) min_y: f64,
    pub(crate) max_x: f64,
    pub(crate) max_y: f64,
}

impl Box {
    fn new(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Self {
        Self { min_x, min_y, max_x, max_y }
    }
}

/// Hilbert R-tree for spatial queries - following flatbush algorithm
///
/// Memory layout (in single buffer):
/// - Header: 8 bytes (magic, version, `node_size`, `num_items`)
/// - All boxes: `num_total_nodes` * 32 bytes (4 f64 per box)
/// - All indices: `num_total_nodes` * 4 bytes (u32 per node)
///
/// Leaf nodes occupy positions [0, `num_items`), parent nodes appended after.
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
    /// Total nodes in tree (cached from level_bounds.last())
    total_nodes: usize,
    /// Pre-allocated capacity in bytes (0 if not pre-allocated)
    allocated_capacity: usize,
}

const MAX_HILBERT: u32 = u16::MAX as u32;
const DEFAULT_NODE_SIZE: usize = 16;
const HEADER_SIZE: usize = 8; // bytes

/// Helper: Estimate total nodes in tree given item count
/// For a tree with node_size, total nodes ≈ N + N/node_size + N/node_size^2 + ...
/// This converges to: N * node_size / (node_size - 1)
#[inline]
fn estimate_total_nodes(num_items: usize, node_size: usize) -> usize {
    if num_items == 0 {
        return 0;
    }
    (num_items * node_size) / (node_size - 1) + 1
}

/// Helper: Calculate required buffer size for estimated nodes
#[inline]
fn estimate_buffer_size(num_items: usize, node_size: usize) -> usize {
    let estimated_nodes = estimate_total_nodes(num_items, node_size);
    HEADER_SIZE + estimated_nodes * (size_of::<Box>() + size_of::<u32>())
}

impl HilbertRTree {
    /// Creates a new empty Hilbert R-tree
    pub fn new() -> Self {
        Self::with_capacity(0)
    }

    /// Creates a new Hilbert R-tree with preallocated capacity
    pub fn with_capacity(capacity: usize) -> Self {
        let data = if capacity > 0 {
            let needed_size = estimate_buffer_size(capacity, DEFAULT_NODE_SIZE);
            Vec::with_capacity(needed_size)
        } else {
            Vec::new()
        };
        
        let allocated_capacity = data.capacity();
        
        Self {
            data,
            allocated_capacity,
            level_bounds: Vec::new(),
            node_size: DEFAULT_NODE_SIZE,
            num_items: 0,
            position: 0,
            bounds: Box::new(f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
            total_nodes: 0,
        }
    }

    /// Adds a bounding box to the tree
    pub fn add(&mut self, min_x: f64, min_y: f64, max_x: f64, max_y: f64) {
        // Calculate required size for this item
        let required_size = estimate_buffer_size(self.num_items + 1, self.node_size);
        
        // Allocate if needed
        if required_size > self.data.capacity() {
            let new_capacity = (self.data.capacity() * 2).max(required_size);
            self.data.reserve(new_capacity - self.data.capacity());
            self.allocated_capacity = self.data.capacity();
        }
        
        // Ensure len is sufficient for writing at the position we need
        let box_idx = HEADER_SIZE + self.num_items * size_of::<Box>();
        if box_idx + size_of::<Box>() > self.data.len() {
            self.data.resize(box_idx + size_of::<Box>(), 0);
        }
        
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
            count = count.div_ceil(node_size);
            total_nodes += count;
            level_bounds.push(total_nodes);
            if count <= 1 {
                break;
            }
        }

        // Resize data buffer to final size
        let data_size = HEADER_SIZE + total_nodes * (size_of::<Box>() + size_of::<u32>());
        if data_size > self.data.capacity() {
            self.data.reserve(data_size - self.data.capacity());
            self.allocated_capacity = self.data.capacity();
        }
        // Ensure len is sufficient for all writes during build
        if self.data.len() < data_size {
            self.data.resize(data_size, 0);
        }

        // Write header
        self.data[0] = 0xfb; // magic
        self.data[1] = 0x01; // version 1 + double type (8)
        self.data[2..4].copy_from_slice(&(node_size as u16).to_le_bytes());
        self.data[4..8].copy_from_slice(&(num_items as u32).to_le_bytes());

        self.level_bounds = level_bounds;
        self.position = 0;
        self.total_nodes = total_nodes;

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
                std::ptr::write_unaligned(root_idx_ptr, 0_u32 << 2_u32); // First child at position 0
            }
            return;
        }

        // Compute Hilbert values for leaves
        let hilbert_width = MAX_HILBERT as f64 / (self.bounds.max_x - self.bounds.min_x);
        let hilbert_height = MAX_HILBERT as f64 / (self.bounds.max_y - self.bounds.min_y);

        let mut hilbert_values = vec![0_u32; num_items];
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
        let mut pos = 0_usize;
        for level_idx in 0..self.level_bounds.len() - 1 {
            let level_end = self.level_bounds[level_idx];
            let mut parent_pos = level_end;

            while pos < level_end {
                let node_index = (pos as u32) << 2_u32; // for JS compatibility
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


    /// Finds all boxes that intersect with a given rectangular region.
    ///
    /// This query returns all boxes whose bounding boxes overlap with the query rectangle,
    /// including boxes that merely touch at edges or corners. This is useful for broad-phase
    /// collision detection, finding objects in a viewport, or spatial filtering.
    ///
    /// # Arguments
    /// * `min_x` - Left edge of query rectangle
    /// * `min_y` - Bottom edge of query rectangle
    /// * `max_x` - Right edge of query rectangle
    /// * `max_y` - Top edge of query rectangle
    /// * `results` - Output vector; will be cleared and populated with matching box indices
    ///
    /// # Example
    /// ```
    /// use aabb::prelude::*;
    /// let mut tree = AABB::with_capacity(3);
    /// tree.add(0.0, 0.0, 2.0, 2.0);  // Box 0
    /// tree.add(1.0, 1.0, 3.0, 3.0);  // Box 1
    /// tree.add(4.0, 4.0, 5.0, 5.0);  // Box 2
    /// tree.build();
    ///
    /// let mut results = Vec::new();
    /// tree.query_intersecting(0.5, 0.5, 2.5, 2.5, &mut results);
    /// // Results include box 0 and 1 (both intersect the query rectangle)
    /// ```
    pub fn query_intersecting(
        &self,
        min_x: f64,
        min_y: f64,
        max_x: f64,
        max_y: f64,
        results: &mut Vec<usize>,
    ) {
        results.clear();
        if self.num_items == 0 || self.level_bounds.is_empty() {
            return;
        }

        // Query area heuristic for early termination decision
        let query_area = (max_x - min_x) * (max_y - min_y);
        let bounds_area = (self.bounds.max_x - self.bounds.min_x)
            * (self.bounds.max_y - self.bounds.min_y);

        // If query covers >50% of space, full scan is faster than hierarchical traversal
        if query_area > bounds_area * 0.5 {
            // Fast path: scan all leaf nodes directly
            for pos in 0..self.num_items {
                let node_box = self.get_box(pos);

                if max_x >= node_box.min_x && max_y >= node_box.min_y
                    && min_x <= node_box.max_x && min_y <= node_box.max_y
                {
                    let index = self.get_index(pos);
                    results.push(index as usize);
                }
            }
            return;
        }

        // Slow path: hierarchical traversal with pruning
        let mut queue = VecDeque::new();
        let mut node_index = self.total_nodes - 1;

        loop {
            // Find bounds of current level
            let node_end = self.upper_bound(node_index);
            let end_pos = (node_index + self.node_size).min(node_end);

            // Process groups of 4 nodes at a time
            let mut pos = node_index;
            while pos + 4 <= end_pos {
                let boxes = self.get_boxes_batch(pos);
                
                for i in 0..4 {
                    let node_box = boxes[i];
                    if !(max_x < node_box.min_x || max_y < node_box.min_y
                        || min_x > node_box.max_x || min_y > node_box.max_y)
                    {
                        let current_pos = pos + i;
                        let index = self.get_index(current_pos);
                        if current_pos < self.num_items {
                            results.push(index as usize);
                        } else {
                            queue.push_back((index >> 2) as usize);
                        }
                    }
                }
                
                pos += 4;
            }
            
            // Process remaining nodes (1-3) individually
            while pos < end_pos {
                let node_box = self.get_box(pos);
                if !(max_x < node_box.min_x || max_y < node_box.min_y
                    || min_x > node_box.max_x || min_y > node_box.max_y)
                {
                    let index = self.get_index(pos);
                    if pos < self.num_items {
                        results.push(index as usize);
                    } else {
                        queue.push_back((index >> 2) as usize);
                    }
                }
                pos += 1;
            }

            if queue.is_empty() {
                break;
            }

            node_index = queue.pop_front().unwrap();
        }
    }

    /// Finds the K nearest boxes to a point using Euclidean distance.
    ///
    /// This query efficiently finds the K closest boxes to a given point by computing
    /// the distance from the point to each box (distance to nearest point in box).
    /// Boxes containing the point have distance 0. This is useful for finding nearby
    /// objects, KNN queries, or closest match lookups.
    ///
    /// # Arguments
    /// * `point_x` - X coordinate of the query point
    /// * `point_y` - Y coordinate of the query point
    /// * `k` - Number of nearest boxes to find
    /// * `results` - Output vector; will be cleared and populated with K nearest box indices,
    ///              sorted by distance (closest first)
    ///
    /// # Example
    /// ```
    /// use aabb::prelude::*;
    /// let mut tree = AABB::with_capacity(3);
    /// tree.add(0.0, 0.0, 1.0, 1.0);  // Box 0
    /// tree.add(2.0, 2.0, 3.0, 3.0);  // Box 1
    /// tree.add(5.0, 5.0, 6.0, 6.0);  // Box 2
    /// tree.build();
    ///
    /// let mut results = Vec::new();
    /// tree.query_nearest_k(0.0, 0.0, 2, &mut results);
    /// // Results contain the 2 nearest boxes
    /// ```
    pub fn query_nearest_k(
        &self,
        point_x: f64,
        point_y: f64,
        k: usize,
        results: &mut Vec<usize>,
    ) {
        results.clear();
        if self.num_items == 0 || self.level_bounds.is_empty() || k == 0 {
            return;
        }

        use std::collections::BinaryHeap;
        use std::cmp::Ordering;

        // Priority queue entry: (distance, node_pos, is_leaf)
        // Use reverse ordering so we get min-heap (closest items first)
        #[derive(Debug, Clone, Copy)]
        struct NodeEntry {
            dist_sq: f64,
            pos: usize,
            is_leaf: bool,
        }

        impl Eq for NodeEntry {}
        impl PartialEq for NodeEntry {
            fn eq(&self, other: &Self) -> bool {
                self.dist_sq == other.dist_sq && self.pos == other.pos
            }
        }
        impl Ord for NodeEntry {
            fn cmp(&self, other: &Self) -> Ordering {
                // Reverse order for min-heap: larger distances sort first
                other.dist_sq.partial_cmp(&self.dist_sq)
                    .unwrap_or(Ordering::Equal)
            }
        }
        impl PartialOrd for NodeEntry {
            fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
                Some(self.cmp(other))
            }
        }

        // Result accumulator: use max-heap of (distance, index) to track K nearest
        // When heap size > k, pop the farthest element
        #[derive(Debug, Clone, Copy)]
        struct ResultEntry {
            dist_sq: f64,
            idx: u32,
        }

        impl Eq for ResultEntry {}
        impl PartialEq for ResultEntry {
            fn eq(&self, other: &Self) -> bool {
                self.dist_sq == other.dist_sq
            }
        }
        impl Ord for ResultEntry {
            fn cmp(&self, other: &Self) -> Ordering {
                // Forward order for max-heap: smaller distances sort first
                // This way we keep the K smallest distances
                self.dist_sq.partial_cmp(&other.dist_sq)
                    .unwrap_or(Ordering::Equal)
            }
        }
        impl PartialOrd for ResultEntry {
            fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
                Some(self.cmp(other))
            }
        }

        let mut queue = BinaryHeap::new();
        let mut result_heap = BinaryHeap::new();
        
        // Start at root level (highest level has fewest nodes)
        let root_level = self.level_bounds.len() - 1;
        let root_start = if root_level > 0 {
            self.level_bounds[root_level - 1]
        } else {
            0
        };
        let root_end = self.level_bounds[root_level];

        // Initialize queue with root nodes
        for pos in root_start..root_end {
            let node_box = self.get_box(pos);
            let dx = self.axis_distance(point_x, node_box.min_x, node_box.max_x);
            let dy = self.axis_distance(point_y, node_box.min_y, node_box.max_y);
            let dist_sq = dx * dx + dy * dy;
            queue.push(NodeEntry {
                dist_sq,
                pos,
                is_leaf: false,
            });
        }

        let mut max_dist_sq = f64::INFINITY;

        // Traverse tree in priority order
        while let Some(entry) = queue.pop() {
            // Skip if this node is farther than our kth result
            if entry.dist_sq > max_dist_sq {
                // Early exit: once we have k results and current node is too far,
                // all remaining nodes in the priority queue are even farther
                if result_heap.len() == k {
                    break;
                }
                continue;
            }

            if entry.is_leaf {
                // This is a leaf item - add to results
                let index = self.get_index(entry.pos);
                result_heap.push(ResultEntry {
                    dist_sq: entry.dist_sq,
                    idx: index,
                });

                // Keep only k results, removing farthest if we exceed k
                if result_heap.len() > k {
                    result_heap.pop();
                }

                // Update max distance threshold
                if result_heap.len() == k
                    && let Some(&top) = result_heap.peek()
                {
                    max_dist_sq = top.dist_sq;
                }
            } else {
                // Internal node - add its children to queue
                // Children span from level_bounds[level] to level_bounds[level+1]
                
                // Find which level this node is at
                let mut level = 0;
                for i in 0..self.level_bounds.len() {
                    if entry.pos < self.level_bounds[i] {
                        level = i;
                        break;
                    }
                }

                if level > 0 {
                    let child_level_start = self.level_bounds[level - 1];
                    let child_level_end = if level == 0 {
                        self.num_items
                    } else {
                        self.level_bounds[level - 1]
                    };

                    // Internal nodes point to their children via indices
                    // Calculate which children belong to this parent node
                    // Node at position in this level has node_size children at next level down
                    let node_index = self.get_index(entry.pos);
                    let first_child = (node_index >> 2) as usize;

                    for child_idx in 0..self.node_size {
                        let child_pos = first_child + child_idx;
                        if child_pos >= child_level_end {
                            break;
                        }

                        let child_box = self.get_box(child_pos);
                        let dx = self.axis_distance(point_x, child_box.min_x, child_box.max_x);
                        let dy = self.axis_distance(point_y, child_box.min_y, child_box.max_y);
                        let dist_sq = dx * dx + dy * dy;

                        // Only add if within threshold or we haven't found k results yet
                        if dist_sq <= max_dist_sq || result_heap.len() < k {
                            let is_child_leaf = child_level_start == self.num_items;
                            queue.push(NodeEntry {
                                dist_sq,
                                pos: child_pos,
                                is_leaf: is_child_leaf,
                            });
                        }
                    }
                } else {
                    // Children are leaf items
                    let first_child = (self.get_index(entry.pos) >> 2) as usize;
                    for child_idx in 0..self.node_size {
                        let child_pos = first_child + child_idx;
                        if child_pos >= self.num_items {
                            break;
                        }

                        let child_box = self.get_box(child_pos);
                        let dx = self.axis_distance(point_x, child_box.min_x, child_box.max_x);
                        let dy = self.axis_distance(point_y, child_box.min_y, child_box.max_y);
                        let dist_sq = dx * dx + dy * dy;

                        if dist_sq <= max_dist_sq || result_heap.len() < k {
                            queue.push(NodeEntry {
                                dist_sq,
                                pos: child_pos,
                                is_leaf: true,
                            });
                        }
                    }
                }
            }
        }

        // Extract results and sort by distance (ascending)
        let mut distance_pairs: Vec<(f64, u32)> = result_heap
            .into_iter()
            .map(|entry| (entry.dist_sq, entry.idx))
            .collect();
        
        // Sort by distance ascending - this is already a partial sort since we only have k elements
        distance_pairs.sort_unstable_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal));
        
        for (_, idx) in distance_pairs {
            results.push(idx as usize);
        }
    }

    /// Finds all boxes that contain a specific point.
    ///
    /// This query returns all boxes whose boundaries include the given point.
    /// A point on the edge or corner of a box is considered contained (inclusive test).
    /// This is useful for hit testing, picking objects at a screen location, or
    /// determining which regions contain a specific coordinate.
    ///
    /// # Arguments
    /// * `x` - X coordinate of the query point
    /// * `y` - Y coordinate of the query point
    /// * `results` - Output vector; will be cleared and populated with indices of all
    ///              boxes that contain the point
    ///
    /// # Example
    /// ```
    /// use aabb::prelude::*;
    /// let mut tree = AABB::with_capacity(2);
    /// tree.add(0.0, 0.0, 2.0, 2.0);  // Box 0
    /// tree.add(1.0, 1.0, 3.0, 3.0);  // Box 1
    /// tree.build();
    ///
    /// let mut results = Vec::new();
    /// tree.query_point(1.5, 1.5, &mut results);
    /// // Results contain both box 0 and box 1 (point is inside both)
    /// ```
    pub fn query_point(&self, x: f64, y: f64, results: &mut Vec<usize>) {
        results.clear();
        if self.num_items == 0 || self.level_bounds.is_empty() {
            return;
        }
        
        let mut queue = VecDeque::new();
        let mut node_index = self.total_nodes - 1;
        
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
                if pos >= self.num_items {
                    queue.push_back((index >> 2) as usize);
                } else {
                    results.push(index as usize);
                }
            }
            
            if queue.is_empty() {
                break;
            }

            node_index = queue.pop_front().unwrap();
        }
    }

    /// Finds all boxes that completely contain a given rectangular region.
    ///
    /// This query returns all boxes whose boundaries fully enclose the query rectangle.
    /// The query rectangle must be fully contained within each result box for inclusion.
    /// This is useful for finding container regions, parent regions, or areas that
    /// fully cover a given space.
    ///
    /// # Arguments
    /// * `min_x` - Left edge of query rectangle
    /// * `min_y` - Bottom edge of query rectangle
    /// * `max_x` - Right edge of query rectangle
    /// * `max_y` - Top edge of query rectangle
    /// * `results` - Output vector; will be cleared and populated with indices of boxes
    ///              that completely contain the query rectangle
    ///
    /// # Example
    /// ```
    /// use aabb::prelude::*;
    /// let mut tree = AABB::with_capacity(3);
    /// tree.add(0.0, 0.0, 5.0, 5.0);  // Box 0 (large)
    /// tree.add(1.0, 1.0, 4.0, 4.0);  // Box 1 (medium)
    /// tree.add(6.0, 6.0, 8.0, 8.0);  // Box 2 (separate)
    /// tree.build();
    ///
    /// let mut results = Vec::new();
    /// tree.query_contain(1.5, 1.5, 3.5, 3.5, &mut results);
    /// // Results contain box 0 and box 1 (both contain the query rectangle)
    /// ```
    pub fn query_contain(&self, min_x: f64, min_y: f64, max_x: f64, max_y: f64, results: &mut Vec<usize>) {
        results.clear();
        if self.num_items == 0 || self.level_bounds.is_empty() {
            return;
        }
        
        let mut queue = VecDeque::new();
        let mut node_index = self.total_nodes - 1;
        
        loop {
            let node_end = self.upper_bound(node_index);
            let end_pos = (node_index + self.node_size).min(node_end);
            
            for pos in node_index..end_pos {
                let node_box = self.get_box(pos);
                
                // Check if node contains the query rectangle
                if node_box.min_x <= min_x && node_box.max_x >= max_x &&
                   node_box.min_y <= min_y && node_box.max_y >= max_y {
                    
                    let index = self.get_index(pos);
                    if pos >= self.num_items {
                        queue.push_back((index >> 2) as usize);
                    } else {
                        results.push(index as usize);
                    }
                }
            }
            
            if queue.is_empty() {
                break;
            }

            node_index = queue.pop_front().unwrap();
        }
    }

    /// Finds all boxes that are completely contained within a given rectangle.
    ///
    /// This query returns all boxes that fit entirely within the query rectangle's boundaries.
    /// Each result box must be fully contained for inclusion. This is the opposite of
    /// `query_contain` and is useful for finding items within a region, filtering
    /// objects by area, or identifying sub-regions.
    ///
    /// # Arguments
    /// * `min_x` - Left edge of query rectangle
    /// * `min_y` - Bottom edge of query rectangle
    /// * `max_x` - Right edge of query rectangle
    /// * `max_y` - Top edge of query rectangle
    /// * `results` - Output vector; will be cleared and populated with indices of boxes
    ///              that are completely contained within the query rectangle
    ///
    /// # Example
    /// ```
    /// use aabb::prelude::*;
    /// let mut tree = AABB::with_capacity(3);
    /// tree.add(0.0, 0.0, 5.0, 5.0);  // Box 0 (large)
    /// tree.add(1.0, 1.0, 2.0, 2.0);  // Box 1 (small, inside)
    /// tree.add(6.0, 6.0, 8.0, 8.0);  // Box 2 (outside)
    /// tree.build();
    ///
    /// let mut results = Vec::new();
    /// tree.query_contained_within(0.5, 0.5, 4.5, 4.5, &mut results);
    /// // Results contain only box 1 (box 0 is too large, box 2 is outside)
    /// ```
    pub fn query_contained_within(&self, min_x: f64, min_y: f64, max_x: f64, max_y: f64, results: &mut Vec<usize>) {
        results.clear();
        if self.num_items == 0 || self.level_bounds.is_empty() {
            return;
        }
        
        let mut queue = VecDeque::new();
        let mut node_index = self.total_nodes - 1;
        
        loop {
            let node_end = self.upper_bound(node_index);
            let end_pos = (node_index + self.node_size).min(node_end);
            
            for pos in node_index..end_pos {
                let node_box = self.get_box(pos);
                
                if pos >= self.num_items {
                    // This is a parent node - check if it could have matching children
                    // (any overlap with query region)
                    if node_box.max_x >= min_x && node_box.max_y >= min_y &&
                       node_box.min_x <= max_x && node_box.min_y <= max_y {
                        let index = self.get_index(pos);
                        queue.push_back((index >> 2) as usize);
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

            node_index = queue.pop_front().unwrap();
        }
    }

    /// Finds the first K intersecting boxes within a rectangular region.
    ///
    /// This query finds boxes intersecting a query rectangle and stops after collecting K results.
    /// Unlike `query_intersecting` which finds all matches, this variant returns early when K
    /// results are found, making it more efficient when only a limited number of results are needed.
    /// Results are included in the order they are encountered during tree traversal.
    ///
    /// # Arguments
    /// * `min_x` - Left edge of query rectangle
    /// * `min_y` - Bottom edge of query rectangle
    /// * `max_x` - Right edge of query rectangle
    /// * `max_y` - Top edge of query rectangle
    /// * `k` - Maximum number of results to return
    /// * `results` - Output vector; will be cleared and populated with up to K matching box indices
    ///
    /// # Example
    /// ```
    /// use aabb::prelude::*;
    /// let mut tree = AABB::with_capacity(5);
    /// tree.add(0.0, 0.0, 1.0, 1.0);  // Box 0
    /// tree.add(0.5, 0.5, 1.5, 1.5);  // Box 1
    /// tree.add(1.0, 1.0, 2.0, 2.0);  // Box 2
    /// tree.add(1.5, 1.5, 2.5, 2.5);  // Box 3
    /// tree.add(4.0, 4.0, 5.0, 5.0);  // Box 4
    /// tree.build();
    ///
    /// let mut results = Vec::new();
    /// tree.query_intersecting_k(0.0, 0.0, 2.0, 2.0, 2, &mut results);
    /// // Results contain at most 2 of the intersecting boxes
    /// ```
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
        
        let mut queue = VecDeque::with_capacity(self.level_bounds.len() * 2);
        let mut node_index = self.total_nodes - 1;
        
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
                    queue.push_back((index >> 2) as usize);
                }
            }
            
            if queue.is_empty() {
                break;
            }

            node_index = queue.pop_front().unwrap();
        }
    }

    /// Finds all boxes that intersect with a circular region.
    ///
    /// This query returns all boxes whose bounding boxes intersect with the circle
    /// centered at `(center_x, center_y)` with the given `radius`. The distance check
    /// uses the Euclidean distance from the circle's center to the nearest point in each box.
    /// This is useful for circular range queries, area-of-effect searches, and radial filtering.
    ///
    /// # Arguments
    /// * `center_x` - X coordinate of circle center
    /// * `center_y` - Y coordinate of circle center
    /// * `radius` - Radius of the circular region
    /// * `results` - Output vector; will be cleared and populated with indices of all boxes
    ///              intersecting the circular region
    ///
    /// # Example
    /// ```
    /// use aabb::prelude::*;
    /// let mut tree = AABB::with_capacity(3);
    /// tree.add(0.0, 0.0, 1.0, 1.0);  // Box 0
    /// tree.add(1.0, 1.0, 2.0, 2.0);  // Box 1
    /// tree.add(5.0, 5.0, 6.0, 6.0);  // Box 2
    /// tree.build();
    ///
    /// let mut results = Vec::new();
    /// tree.query_circle(1.0, 1.0, 1.5, &mut results);
    /// // Results include boxes 0 and 1 (within circle), but not box 2
    /// ```
    pub fn query_circle(&self, center_x: f64, center_y: f64, radius: f64, results: &mut Vec<usize>) {
        if self.num_items == 0 || self.level_bounds.is_empty() || radius < 0.0 {
            results.clear();
            return;
        }

        results.clear();
        
        let radius_sq = radius * radius;
        let mut queue = VecDeque::new();
        let mut node_index = self.total_nodes - 1;
        
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
                    if pos >= self.num_items {
                        queue.push_back((index >> 2) as usize);
                    } else {
                        results.push(index as usize);
                    }
                }
            }
            
            if queue.is_empty() {
                break;
            }

            node_index = queue.pop_front().unwrap();
        }
    }

    /// Finds all boxes that intersect with a rectangle's directional movement path.
    ///
    /// This query simulates moving a rectangle from its initial position in a given direction
    /// for a specified distance, and returns all boxes intersecting the swept path. The sweep
    /// region is the bounding box of the rectangle at its start and end positions.
    /// This is useful for collision detection along movement paths, ray casting, and
    /// predictive spatial queries (e.g., finding obstacles in a moving object's path).
    ///
    /// # Arguments
    /// * `min_x` - Left edge of the rectangle
    /// * `min_y` - Bottom edge of the rectangle
    /// * `max_x` - Right edge of the rectangle
    /// * `max_y` - Top edge of the rectangle
    /// * `dir_x` - X component of movement direction vector
    /// * `dir_y` - Y component of movement direction vector
    /// * `distance` - Distance to move in the direction (direction is normalized internally)
    /// * `results` - Output vector; will be cleared and populated with indices of all boxes
    ///              intersecting the swept movement path
    ///
    /// # Example
    /// ```
    /// use aabb::prelude::*;
    /// let mut tree = AABB::with_capacity(3);
    /// tree.add(0.0, 0.0, 1.0, 1.0);  // Box 0
    /// tree.add(3.0, 0.0, 4.0, 1.0);  // Box 1 (in the path)
    /// tree.add(5.0, 5.0, 6.0, 6.0);  // Box 2 (not in the path)
    /// tree.build();
    ///
    /// let mut results = Vec::new();
    /// // Move rectangle from (0,0)-(1,1) to the right for distance 3
    /// tree.query_in_direction(0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 3.0, &mut results);
    /// // Results include boxes 0 and 1
    /// ```
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
        results.clear();
        if self.num_items == 0 || self.level_bounds.is_empty() || distance < 0.0 {
            return;
        }
        
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

        let mut queue = VecDeque::new();
        let mut node_index = self.total_nodes - 1;
        
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
                if pos >= self.num_items {
                    queue.push_back((index >> 2) as usize);
                } else {
                    results.push(index as usize);
                }
            }
            
            if queue.is_empty() {
                break;
            }
            
            node_index = queue.pop_front().unwrap();
        }
    }

    /// Finds the K nearest boxes intersecting a rectangle's directional movement path.
    ///
    /// This query finds boxes that intersect with a moving rectangle's swept path and
    /// returns the K closest ones ordered by distance along the movement direction.
    /// Distance is computed as the projection of each box's center onto the normalized
    /// direction vector, making results ordered by "how far along the path" each box is.
    /// This is useful for ordered collision detection, finding the closest obstacles
    /// in a movement path, or sequential hit detection in a sweep.
    ///
    /// # Arguments
    /// * `min_x` - Left edge of the rectangle
    /// * `min_y` - Bottom edge of the rectangle
    /// * `max_x` - Right edge of the rectangle
    /// * `max_y` - Top edge of the rectangle
    /// * `dir_x` - X component of movement direction vector
    /// * `dir_y` - Y component of movement direction vector
    /// * `k` - Maximum number of results to return
    /// * `distance` - Distance to move in the direction (direction is normalized internally)
    /// * `results` - Output vector; will be cleared and populated with up to K nearest box indices
    ///              sorted by distance along the movement direction
    ///
    /// # Example
    /// ```
    /// use aabb::prelude::*;
    /// let mut tree = AABB::with_capacity(4);
    /// tree.add(0.0, 0.0, 1.0, 1.0);  // Box 0
    /// tree.add(2.0, 0.0, 3.0, 1.0);  // Box 1 (first in path)
    /// tree.add(4.0, 0.0, 5.0, 1.0);  // Box 2 (second in path)
    /// tree.add(6.0, 6.0, 7.0, 7.0);  // Box 3 (not in path)
    /// tree.build();
    ///
    /// let mut results = Vec::new();
    /// // Move rectangle right, find 2 nearest obstacles
    /// tree.query_in_direction_k(0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 2, 10.0, &mut results);
    /// // Results contain boxes 1 and 2 (ordered by distance along direction)
    /// ```
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
        queue.push(self.total_nodes - 1);

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

        // Partial sort: get K smallest elements by distance
        results.clear();
        
        if candidates.len() <= k {
            // Fewer candidates than K, just sort all
            candidates.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
            for (_, idx) in candidates {
                results.push(idx);
            }
        } else {
            // More candidates than K, use partial sort
            candidates.select_nth_unstable_by(k - 1, |a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
            // Now split at position k and sort only the K elements
            let (k_smallest, _) = candidates.split_at_mut(k);
            k_smallest.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
            for (_, idx) in k_smallest {
                results.push(*idx);
            }
        }
    }

    /// Get box at position using read_unaligned
    #[inline(always)]
    pub(crate) fn get_box(&self, pos: usize) -> Box {
        let idx = HEADER_SIZE + pos * size_of::<Box>();
        unsafe {
            std::ptr::read_unaligned(&self.data[idx] as *const u8 as *const Box)
        }
    }

    /// Get 4 boxes at once for batch processing - single read_unaligned call
    #[allow(dead_code)]
    #[inline]
    pub(crate) fn get_boxes_batch(&self, start_pos: usize) -> [Box; 4] {
        let base_idx = HEADER_SIZE + start_pos * size_of::<Box>();
        unsafe {
            std::ptr::read_unaligned(self.data.as_ptr().add(base_idx) as *const [Box; 4])
        }
    }

    /// Get index at position using read_unaligned
    #[inline(always)]
    pub(crate) fn get_index(&self, pos: usize) -> u32 {
        let indices_start = HEADER_SIZE + self.total_nodes * size_of::<Box>();
        unsafe {
            std::ptr::read_unaligned(&self.data[indices_start + pos * size_of::<u32>()] as *const u8 as *const u32)
        }
    }

    /// Get distance along an axis
    #[inline(always)]
    fn axis_distance(&self, coordinate: f64, min: f64, max: f64) -> f64 {
        if coordinate < min {
            min - coordinate
        } else if coordinate > max {
            coordinate - max
        } else {
            0.0
        }
    }

    /// Find upper bound of a node in `level_bounds`
    #[inline(always)]
    fn upper_bound(&self, node_index: usize) -> usize {
        // Binary search: find first level_bound > node_index
        let idx = self.level_bounds.partition_point(|&bound| bound <= node_index);
        if idx < self.level_bounds.len() {
            self.level_bounds[idx]
        } else {
            self.total_nodes
        }
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
    #[inline]
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
        let indices_start = HEADER_SIZE + self.total_nodes * size_of::<Box>();
        
        let left_idx_ptr = &mut self.data[indices_start + left * size_of::<u32>()] as *mut u8 as *mut u32;
        let right_idx_ptr = &mut self.data[indices_start + right * size_of::<u32>()] as *mut u8 as *mut u32;
        
        unsafe {
            let left_idx = std::ptr::read_unaligned(left_idx_ptr);
            let right_idx = std::ptr::read_unaligned(right_idx_ptr);
            std::ptr::write_unaligned(left_idx_ptr, right_idx);
            std::ptr::write_unaligned(right_idx_ptr, left_idx);
        }
    }

    /// Saves the built Hilbert R-tree to a file.
    ///
    /// Serializes the complete tree structure including the header, buffer, metadata, and level bounds
    /// to enable fast loading without rebuilding. The file format includes a magic number and version
    /// for integrity checking during load.
    ///
    /// # Arguments
    /// * `path` - File path where the tree will be saved
    ///
    /// # Errors
    /// Returns an error if the file cannot be created or written to.
    ///
    /// # Example
    /// ```ignore
    /// let mut tree = HilbertRTree::with_capacity(3);
    /// tree.add(0.0, 0.0, 1.0, 1.0);
    /// tree.add(1.0, 1.0, 2.0, 2.0);
    /// tree.build();
    /// tree.save("my_tree.bin")?;
    /// ```
    pub fn save<P: AsRef<std::path::Path>>(&self, path: P) -> std::io::Result<()> {
        use std::io::Write;
        let mut file = std::fs::File::create(path)?;
        
        // Write magic number and version (file header for validation)
        file.write_all(&[0xfb])?;  // magic (f64 variant)
        file.write_all(&[0x01])?;  // version 1 (f64 variant)
        
        // Write node_size
        file.write_all(&(self.node_size as u32).to_le_bytes())?;
        
        // Write num_items
        file.write_all(&(self.num_items as u32).to_le_bytes())?;
        
        // Write total_nodes
        file.write_all(&(self.total_nodes as u32).to_le_bytes())?;
        
        // Write level_bounds length
        file.write_all(&(self.level_bounds.len() as u32).to_le_bytes())?;
        
        // Write level_bounds
        for &bound in &self.level_bounds {
            file.write_all(&(bound as u32).to_le_bytes())?;
        }
        
        // Write bounds
        file.write_all(&self.bounds.min_x.to_le_bytes())?;
        file.write_all(&self.bounds.min_y.to_le_bytes())?;
        file.write_all(&self.bounds.max_x.to_le_bytes())?;
        file.write_all(&self.bounds.max_y.to_le_bytes())?;
        
        // Write data buffer
        file.write_all(&(self.data.len() as u32).to_le_bytes())?;
        file.write_all(&self.data)?;
        
        Ok(())
    }

    /// Loads a Hilbert R-tree from a file.
    ///
    /// Deserializes a tree that was previously saved with `save()`.
    /// Validates the file format by checking the magic number and version.
    /// The loaded tree is immediately ready for querying without rebuilding.
    ///
    /// # Arguments
    /// * `path` - File path of the saved tree
    ///
    /// # Errors
    /// Returns an error if the file cannot be read, the format is invalid,
    /// or the magic number/version check fails.
    ///
    /// # Example
    /// ```ignore
    /// let tree = HilbertRTree::load("my_tree.bin")?;
    /// let mut results = Vec::new();
    /// tree.query_intersecting(0.0, 0.0, 1.0, 1.0, &mut results);
    /// ```
    pub fn load<P: AsRef<std::path::Path>>(path: P) -> std::io::Result<Self> {
        use std::io::Read;
        let mut file = std::fs::File::open(path)?;
        
        // Read and validate magic number
        let mut magic_buf = [0u8; 1];
        file.read_exact(&mut magic_buf)?;
        if magic_buf[0] != 0xfb {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "Invalid file format: magic number mismatch",
            ));
        }
        
        // Read and validate version
        let mut version_buf = [0u8; 1];
        file.read_exact(&mut version_buf)?;
        if version_buf[0] != 0x01 {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "Unsupported file version (expected f64 variant v1, got different version)",
            ));
        }
        
        // Read node_size
        let mut buf = [0u8; 4];
        file.read_exact(&mut buf)?;
        let node_size = u32::from_le_bytes(buf) as usize;
        
        // Read num_items
        file.read_exact(&mut buf)?;
        let num_items = u32::from_le_bytes(buf) as usize;
        
        // Read total_nodes
        file.read_exact(&mut buf)?;
        let total_nodes = u32::from_le_bytes(buf) as usize;
        
        // Read level_bounds length
        file.read_exact(&mut buf)?;
        let level_bounds_len = u32::from_le_bytes(buf) as usize;
        
        // Read level_bounds
        let mut level_bounds = Vec::with_capacity(level_bounds_len);
        for _ in 0..level_bounds_len {
            file.read_exact(&mut buf)?;
            level_bounds.push(u32::from_le_bytes(buf) as usize);
        }
        
        // Read bounds
        let mut f64_buf = [0u8; 8];
        file.read_exact(&mut f64_buf)?;
        let min_x = f64::from_le_bytes(f64_buf);
        file.read_exact(&mut f64_buf)?;
        let min_y = f64::from_le_bytes(f64_buf);
        file.read_exact(&mut f64_buf)?;
        let max_x = f64::from_le_bytes(f64_buf);
        file.read_exact(&mut f64_buf)?;
        let max_y = f64::from_le_bytes(f64_buf);
        
        let bounds = Box::new(min_x, min_y, max_x, max_y);
        
        // Read data buffer
        file.read_exact(&mut buf)?;
        let data_len = u32::from_le_bytes(buf) as usize;
        let mut data = vec![0u8; data_len];
        file.read_exact(&mut data)?;
        
        Ok(Self {
            data,
            level_bounds,
            node_size,
            num_items,
            position: 0,
            bounds,
            total_nodes,
            allocated_capacity: data_len,
        })
    }
}

impl Default for HilbertRTree {
    fn default() -> Self {
        Self::new()
    }
}

/// Hilbert curve index computation
/// From <https://github.com/rawrunprotected/hilbert_curves> (public domain)
#[inline(always)]
fn interleave(mut x: u32) -> u32 {
    x = (x | (x << 8)) & 0x00FF00FF;
    x = (x | (x << 4)) & 0x0F0F0F0F;
    x = (x | (x << 2)) & 0x33333333;
    x = (x | (x << 1)) & 0x55555555;
    x
}

#[expect(non_snake_case)]
#[inline]
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
