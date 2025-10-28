//#![warn(unsafe_code)]

//! Hilbert R-tree implementation for i32 coordinates using unsafe memory layout for performance.
//!
//! All unsafe operations are internal implementation details. The public API is safe.
//! Memory is managed in a single buffer with type-punned box structures and indices.
//! Buffer invariants are maintained throughout the tree's lifetime.
//!
//! This implementation supports i32 integer coordinates, providing 50% better memory efficiency
//! compared to the f64 version while maintaining the same API for supported queries.
//! 
//! **Supported Queries** (pure AABB operations, no distance calculations):
//! - `query_intersecting` - Find boxes intersecting a rectangle
//! - `query_intersecting_k` - Find first K intersecting boxes
//! - `query_point` - Find boxes containing a point
//! - `query_contain` - Find boxes that contain a rectangle
//! - `query_contained_within` - Find boxes contained within a rectangle

use std::mem::size_of;

/// Box structure: minX, minY, maxX, maxY (16 bytes total for i32)
#[derive(Clone, Copy, Debug)]
pub(crate) struct BoxI32 {
    pub(crate) min_x: i32,
    pub(crate) min_y: i32,
    pub(crate) max_x: i32,
    pub(crate) max_y: i32,
}

impl BoxI32 {
    fn new(min_x: i32, min_y: i32, max_x: i32, max_y: i32) -> Self {
        Self { min_x, min_y, max_x, max_y }
    }
}

/// Hilbert R-tree for i32 spatial queries - following flatbush algorithm
///
/// Memory layout (in single buffer):
/// - Header: 8 bytes (magic, version, `node_size`, `num_items`)
/// - All boxes: `num_total_nodes` * 16 bytes (4 i32 per box) - 50% more efficient than f64!
/// - All indices: `num_total_nodes` * 4 bytes (u32 per node)
///
/// Leaf nodes occupy positions [0, `num_items`), parent nodes appended after.
/// Tree is built bottom-up with Hilbert curve ordering for spatial locality.
#[derive(Clone, Debug)]
pub struct HilbertRTreeI32 {
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
    pub(crate) bounds: BoxI32,
    /// Total nodes in tree (cached from level_bounds.last())
    total_nodes: usize,
}

const MAX_HILBERT: u32 = u16::MAX as u32;
const DEFAULT_NODE_SIZE: usize = 16;
const HEADER_SIZE: usize = 8; // bytes

impl HilbertRTreeI32 {
    /// Creates a new empty Hilbert R-tree for i32 coordinates
    ///
    /// # Example
    /// ```
    /// use aabb::HilbertRTreeI32;
    /// let tree = HilbertRTreeI32::new();
    /// assert_eq!(tree.len(), 0);
    /// ```
    pub fn new() -> Self {
        Self::with_capacity(0)
    }

    /// Creates a new Hilbert R-tree with preallocated capacity
    ///
    /// Preallocating capacity can improve performance by avoiding internal reallocations
    /// during the add phase if you know the approximate number of boxes in advance.
    ///
    /// # Arguments
    /// * `capacity` - Initial capacity for the number of boxes to add
    ///
    /// # Example
    /// ```
    /// use aabb::HilbertRTreeI32;
    /// let mut tree = HilbertRTreeI32::with_capacity(100);
    /// assert_eq!(tree.len(), 0);
    /// ```
    pub fn with_capacity(capacity: usize) -> Self {
        let data = if capacity > 0 {
            let v = vec![0; HEADER_SIZE + capacity * size_of::<BoxI32>()];
            v
        } else {
            Vec::new()
        };
        
        Self {
            data,
            level_bounds: Vec::new(),
            node_size: DEFAULT_NODE_SIZE,
            num_items: 0,
            position: 0,
            bounds: BoxI32::new(i32::MAX, i32::MAX, i32::MIN, i32::MIN),
            total_nodes: 0,
        }
    }

    /// Adds a bounding box to the tree
    ///
    /// Boxes are stored temporarily and reorganized during the `build()` phase.
    /// You must call `build()` before performing any queries.
    ///
    /// # Arguments
    /// * `min_x` - Left edge of the bounding box
    /// * `min_y` - Bottom edge of the bounding box
    /// * `max_x` - Right edge of the bounding box
    /// * `max_y` - Top edge of the bounding box
    ///
    /// # Example
    /// ```
    /// use aabb::HilbertRTreeI32;
    /// let mut tree = HilbertRTreeI32::new();
    /// tree.add(0, 0, 10, 10);  // Box 0
    /// tree.add(5, 5, 15, 15);  // Box 1
    /// tree.build();
    /// assert_eq!(tree.len(), 2);
    /// ```
    pub fn add(&mut self, min_x: i32, min_y: i32, max_x: i32, max_y: i32) {
        // Store boxes temporarily - will be reorganized during build
        if self.num_items == 0 {
            // Pre-allocate data buffer on first add - need space for header + boxes
            let capacity = 128; // Start with reasonable capacity
            self.data.resize(HEADER_SIZE + capacity * size_of::<BoxI32>(), 0);
        }

        // Check if we need to expand
        let required_size = HEADER_SIZE + (self.num_items + 1) * size_of::<BoxI32>();
        if required_size > self.data.len() {
            self.data.resize((self.data.len() * 2).max(required_size), 0);
        }

        let box_idx = HEADER_SIZE + self.num_items * size_of::<BoxI32>();
        let box_ptr = &mut self.data[box_idx] as *mut u8 as *mut BoxI32;
        unsafe {
            std::ptr::write_unaligned(box_ptr, BoxI32::new(min_x, min_y, max_x, max_y));
        }

        self.bounds.min_x = self.bounds.min_x.min(min_x);
        self.bounds.min_y = self.bounds.min_y.min(min_y);
        self.bounds.max_x = self.bounds.max_x.max(max_x);
        self.bounds.max_y = self.bounds.max_y.max(max_y);

        self.num_items += 1;
    }

    /// Builds the Hilbert R-tree index
    ///
    /// This method must be called after adding all boxes and before performing any queries.
    /// It organizes the boxes into a hierarchical structure for efficient spatial queries,
    /// sorting them by their Hilbert curve index for improved cache locality.
    ///
    /// # Performance
    /// Building is O(n log n) due to the sorting phase. After building, queries are O(log n)
    /// on average for well-distributed data.
    ///
    /// # Panics
    /// Does not panic if called on an empty tree or if called multiple times.
    ///
    /// # Example
    /// ```
    /// use aabb::HilbertRTreeI32;
    /// let mut tree = HilbertRTreeI32::new();
    /// tree.add(0, 0, 10, 10);
    /// tree.add(5, 5, 15, 15);
    /// tree.build();  // Required before querying
    /// ```
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
        let data_size = HEADER_SIZE + total_nodes * (size_of::<BoxI32>() + size_of::<u32>());
        self.data.resize(data_size, 0);

        // Write header
        self.data[0] = 0xfb; // magic
        self.data[1] = 0x01; // version 1 + i32 type (4)
        self.data[2..4].copy_from_slice(&(node_size as u16).to_le_bytes());
        self.data[4..8].copy_from_slice(&(num_items as u32).to_le_bytes());

        self.level_bounds = level_bounds;
        self.position = 0;
        self.total_nodes = total_nodes;

        // If all items fit in one node, create a root level
        if num_items <= node_size {
            // Initialize all leaf indices first
            let indices_start = HEADER_SIZE + total_nodes * size_of::<BoxI32>();
            for i in 0..num_items {
                let idx_ptr = &mut self.data[indices_start + i * size_of::<u32>()] as *mut u8 as *mut u32;
                unsafe {
                    std::ptr::write_unaligned(idx_ptr, i as u32);
                }
            }
            
            // Write the root node box at position num_items
            let root_idx = HEADER_SIZE + num_items * size_of::<BoxI32>();
            let root_ptr = &mut self.data[root_idx] as *mut u8 as *mut BoxI32;
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
        let hilbert_width = if self.bounds.max_x > self.bounds.min_x {
            MAX_HILBERT as f64 / (self.bounds.max_x - self.bounds.min_x) as f64
        } else {
            0.0
        };
        let hilbert_height = if self.bounds.max_y > self.bounds.min_y {
            MAX_HILBERT as f64 / (self.bounds.max_y - self.bounds.min_y) as f64
        } else {
            0.0
        };

        let mut hilbert_values = vec![0_u32; num_items];
        for i in 0..num_items {
            let box_data = self.get_box(i);
            let center_x = ((box_data.min_x as f64 + box_data.max_x as f64) / 2.0 - self.bounds.min_x as f64) * hilbert_width;
            let center_y = ((box_data.min_y as f64 + box_data.max_y as f64) / 2.0 - self.bounds.min_y as f64) * hilbert_height;
            let hx = center_x.max(0.0).min(MAX_HILBERT as f64 - 1.0) as u32;
            let hy = center_y.max(0.0).min(MAX_HILBERT as f64 - 1.0) as u32;
            hilbert_values[i] = hilbert_xy_to_index(hx, hy);
        }

        // Initialize leaf indices BEFORE sorting
        let indices_start = HEADER_SIZE + total_nodes * size_of::<BoxI32>();
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
                let box_idx = HEADER_SIZE + parent_pos * size_of::<BoxI32>();
                let box_ptr = &mut self.data[box_idx] as *mut u8 as *mut BoxI32;
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

    /// Returns the number of items in the tree
    ///
    /// # Example
    /// ```
    /// use aabb::HilbertRTreeI32;
    /// let mut tree = HilbertRTreeI32::new();
    /// assert_eq!(tree.len(), 0);
    /// tree.add(0, 0, 10, 10);
    /// assert_eq!(tree.len(), 1);
    /// ```
    pub fn len(&self) -> usize {
        self.num_items
    }

    /// Returns whether the tree is empty
    ///
    /// # Example
    /// ```
    /// use aabb::HilbertRTreeI32;
    /// let mut tree = HilbertRTreeI32::new();
    /// assert!(tree.is_empty());
    /// tree.add(0, 0, 10, 10);
    /// assert!(!tree.is_empty());
    /// ```
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
    /// use aabb::HilbertRTreeI32;
    /// let mut tree = HilbertRTreeI32::with_capacity(3);
    /// tree.add(0, 0, 2, 2);  // Box 0
    /// tree.add(1, 1, 3, 3);  // Box 1
    /// tree.add(4, 4, 5, 5);  // Box 2
    /// tree.build();
    ///
    /// let mut results = Vec::new();
    /// tree.query_intersecting(0, 0, 2, 2, &mut results);
    /// // Results include box 0 and 1 (both intersect the query rectangle)
    /// ```
    pub fn query_intersecting(
        &self,
        min_x: i32,
        min_y: i32,
        max_x: i32,
        max_y: i32,
        results: &mut Vec<usize>,
    ) {
        if self.num_items == 0 || self.level_bounds.is_empty() {
            return;
        }

        results.clear();
        
        let mut queue = Vec::new();
        let mut node_index = self.total_nodes - 1;
        
        loop {
            let node_end = self.upper_bound(node_index);
            let end_pos = (node_index + self.node_size).min(node_end);
            
            for pos in node_index..end_pos {
                let node_box = self.get_box(pos);
                
                // Check if node bbox intersects with query bbox
                if max_x < node_box.min_x || max_y < node_box.min_y ||
                   min_x > node_box.max_x || min_y > node_box.max_y {
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

    /// Finds the first K intersecting boxes within a rectangular region.
    ///
    /// This query finds boxes intersecting a query rectangle and stops after collecting K results.
    /// Unlike `query_intersecting` which finds all matches, this variant returns early when K
    /// results are found, making it more efficient when only a limited number of results are needed.
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
    /// use aabb::HilbertRTreeI32;
    /// let mut tree = HilbertRTreeI32::with_capacity(5);
    /// tree.add(0, 0, 1, 1);  // Box 0
    /// tree.add(0, 0, 2, 2);  // Box 1
    /// tree.add(1, 1, 2, 2);  // Box 2
    /// tree.add(1, 1, 3, 3);  // Box 3
    /// tree.add(4, 4, 5, 5);  // Box 4
    /// tree.build();
    ///
    /// let mut results = Vec::new();
    /// tree.query_intersecting_k(0, 0, 2, 2, 2, &mut results);
    /// // Results contain at most 2 of the intersecting boxes
    /// ```
    pub fn query_intersecting_k(
        &self,
        min_x: i32,
        min_y: i32,
        max_x: i32,
        max_y: i32,
        k: usize,
        results: &mut Vec<usize>,
    ) {
        if self.num_items == 0 || self.level_bounds.is_empty() || k == 0 {
            results.clear();
            return;
        }

        results.clear();
        
        let mut queue = Vec::with_capacity(self.level_bounds.len() * 2);
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
                    queue.push((index >> 2) as usize);
                }
            }
            
            if queue.is_empty() {
                break;
            }
            
            node_index = queue.remove(0);
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
    /// use aabb::HilbertRTreeI32;
    /// let mut tree = HilbertRTreeI32::with_capacity(2);
    /// tree.add(0, 0, 2, 2);  // Box 0
    /// tree.add(1, 1, 3, 3);  // Box 1
    /// tree.build();
    ///
    /// let mut results = Vec::new();
    /// tree.query_point(1, 1, &mut results);
    /// // Results contain both box 0 and box 1 (point is inside both)
    /// ```
    pub fn query_point(&self, x: i32, y: i32, results: &mut Vec<usize>) {
        if self.num_items == 0 || self.level_bounds.is_empty() {
            return;
        }

        results.clear();
        
        let mut queue = Vec::new();
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
    /// use aabb::HilbertRTreeI32;
    /// let mut tree = HilbertRTreeI32::with_capacity(3);
    /// tree.add(0, 0, 5, 5);  // Box 0 (large)
    /// tree.add(1, 1, 4, 4);  // Box 1 (medium)
    /// tree.add(6, 6, 8, 8);  // Box 2 (separate)
    /// tree.build();
    ///
    /// let mut results = Vec::new();
    /// tree.query_contain(1, 1, 3, 3, &mut results);
    /// // Results contain box 0 and box 1 (both contain the query rectangle)
    /// ```
    pub fn query_contain(&self, min_x: i32, min_y: i32, max_x: i32, max_y: i32, results: &mut Vec<usize>) {
        if self.num_items == 0 || self.level_bounds.is_empty() {
            return;
        }

        results.clear();
        
        let mut queue = Vec::new();
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
    /// use aabb::HilbertRTreeI32;
    /// let mut tree = HilbertRTreeI32::with_capacity(3);
    /// tree.add(0, 0, 5, 5);  // Box 0 (large)
    /// tree.add(1, 1, 2, 2);  // Box 1 (small, inside)
    /// tree.add(6, 6, 8, 8);  // Box 2 (outside)
    /// tree.build();
    ///
    /// let mut results = Vec::new();
    /// tree.query_contained_within(0, 0, 4, 4, &mut results);
    /// // Results contain only box 1 (box 0 is too large, box 2 is outside)
    /// ```
    pub fn query_contained_within(&self, min_x: i32, min_y: i32, max_x: i32, max_y: i32, results: &mut Vec<usize>) {
        if self.num_items == 0 || self.level_bounds.is_empty() {
            return;
        }

        results.clear();
        
        let mut queue = Vec::new();
        let mut node_index = self.total_nodes - 1;
        
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

    // --- Private helpers ---

    /// Get box at position
    #[inline]
    pub(crate) fn get_box(&self, pos: usize) -> BoxI32 {
        let idx = HEADER_SIZE + pos * size_of::<BoxI32>();
        let box_slice = unsafe {
            std::slice::from_raw_parts(&self.data[idx] as *const u8 as *const BoxI32, 1)
        };
        box_slice[0]
    }

    /// Get index at position
    #[inline(always)]
    pub(crate) fn get_index(&self, pos: usize) -> u32 {
        let indices_start = HEADER_SIZE + self.total_nodes * size_of::<BoxI32>();
        let idx_slice = unsafe {
            std::slice::from_raw_parts(&self.data[indices_start + pos * size_of::<u32>()] as *const u8 as *const u32, 1)
        };
        idx_slice[0]
    }

    /// Find upper bound of a node in `level_bounds`
    #[inline(always)]
    fn upper_bound(&self, node_index: usize) -> usize {
        for &bound in &self.level_bounds {
            if bound > node_index {
                return bound;
            }
        }
        self.total_nodes
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
        let left_box_idx = HEADER_SIZE + left * size_of::<BoxI32>();
        let right_box_idx = HEADER_SIZE + right * size_of::<BoxI32>();
        
        let left_box = self.get_box(left);
        let right_box = self.get_box(right);
        
        let left_ptr = &mut self.data[left_box_idx] as *mut u8 as *mut BoxI32;
        let right_ptr = &mut self.data[right_box_idx] as *mut u8 as *mut BoxI32;
        
        unsafe {
            std::ptr::write_unaligned(left_ptr, right_box);
            std::ptr::write_unaligned(right_ptr, left_box);
        }

        // Swap indices
        let indices_start = HEADER_SIZE + self.total_nodes * size_of::<BoxI32>();
        
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

impl Default for HilbertRTreeI32 {
    fn default() -> Self {
        Self::new()
    }
}

/// Hilbert curve index computation
/// From <https://github.com/rawrunprotected/hilbert_curves> (public domain)
fn interleave(mut x: u32) -> u32 {
    x = (x | (x << 8)) & 0x00FF00FF;
    x = (x | (x << 4)) & 0x0F0F0F0F;
    x = (x | (x << 2)) & 0x33333333;
    x = (x | (x << 1)) & 0x55555555;
    x
}

#[expect(non_snake_case)]
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
