#![allow(dead_code)]

/// Simple Hilbert R-tree for spatial queries
///
/// Stores bounding boxes in a flat collection sorted by Hilbert curve order.
/// Efficient for static or infrequently-modified spatial data.
///
/// # Examples
/// ```
/// use aabb::HilbertRTree;
///
/// let mut tree = HilbertRTree::new();
/// tree.add(0.0, 0.0, 1.0, 1.0);
/// tree.add(0.5, 0.5, 1.5, 1.5);
/// tree.build();
/// 
/// let mut results = Vec::new();
/// tree.query_intersecting(0.7, 0.7, 1.3, 1.3, &mut results);
/// assert_eq!(results.len(), 2); // Both boxes intersect the query
/// ```
#[doc(hidden)]
#[derive(Clone, Debug)]
pub struct HilbertRTreeLeg {
    /// Flat storage: (`min_x`, `min_y`, `max_x`, `max_y`) for each box
    pub(crate) boxes: Vec<(f64, f64, f64, f64)>,
    /// Hilbert indices for sorting
    pub(crate) hilbert_indices: Vec<u64>,
    /// Sort order: mapping from Hilbert-sorted position to original index
    pub(crate) sorted_order: Vec<usize>,
    /// Whether the tree has been built
    pub(crate) built: bool,
}

impl HilbertRTreeLeg {
    /// Creates a new empty Hilbert R-tree
    pub fn new() -> Self {
        Self::with_capacity(0)
    }

    /// Creates a new Hilbert R-tree with preallocated capacity
    ///
    /// # Arguments
    /// * `capacity` - Expected number of bounding boxes to be added
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            boxes: Vec::with_capacity(capacity),
            hilbert_indices: Vec::with_capacity(capacity),
            sorted_order: Vec::with_capacity(capacity),
            built: false,
        }
    }

    /// Adds a bounding box to the tree
    ///
    /// Must call `build()` after adding all boxes before querying
    pub fn add(&mut self, min_x: f64, min_y: f64, max_x: f64, max_y: f64) {
        self.boxes.push((min_x, min_y, max_x, max_y));
        self.built = false;
    }

    /// Builds the Hilbert R-tree index
    ///
    /// Call this after adding all boxes and before querying
    pub fn build(&mut self) {
        if self.boxes.is_empty() {
            return;
        }

        // Compute Hilbert indices for all boxes (by center)
        self.hilbert_indices.clear();
        for &(min_x, min_y, max_x, max_y) in &self.boxes {
            let center_x = (min_x + max_x) / 2.0;
            let center_y = (min_y + max_y) / 2.0;
            let h_idx = hilbert_index(center_x, center_y, 16);
            self.hilbert_indices.push(h_idx);
        }

        // Create sort order
        self.sorted_order = (0..self.boxes.len()).collect();
        self.sorted_order.sort_by_key(|&idx| self.hilbert_indices[idx]);

        self.built = true;
    }

    /// Returns the number of boxes in the tree
    pub fn len(&self) -> usize {
        self.boxes.len()
    }

    /// Returns whether the tree is empty
    pub fn is_empty(&self) -> bool {
        self.boxes.is_empty()
    }
}

impl Default for HilbertRTreeLeg {
    fn default() -> Self {
        Self::new()
    }
}

/// Compute Hilbert curve index for a point
fn hilbert_index(x: f64, y: f64, max_level: u32) -> u64 {
    // Handle infinite or NaN coordinates
    let safe_x = if x.is_finite() { x } else { 0.0 };
    let safe_y = if y.is_finite() { y } else { 0.0 };
    
    let xi = (safe_x.clamp(0.0, 1.0) * ((1_u64 << max_level) as f64)) as u64;
    let yi = (safe_y.clamp(0.0, 1.0) * ((1_u64 << max_level) as f64)) as u64;
    xy_to_hilbert(xi, yi, max_level)
}

/// Convert 2D coordinates to Hilbert curve index
fn xy_to_hilbert(x: u64, y: u64, order: u32) -> u64 {
    if order == 0 {
        return 0;
    }

    let mut d = 0_u64;
    let mut s = 1_u64 << (order.saturating_sub(1).min(63));

    let mut x = x;
    let mut y = y;

    while s > 0 {
        let rx = ((x & s) > 0) as u64;
        let ry = ((y & s) > 0) as u64;
        d = d.saturating_add(s.saturating_mul(s).saturating_mul((3 * rx) ^ ry));

        // Rotate
        if ry == 0 {
            if rx == 1 {
                x = s.saturating_sub(1).saturating_sub(x);
                y = s.saturating_sub(1).saturating_sub(y);
            }
            std::mem::swap(&mut x, &mut y);
        }

        s >>= 1;
    }

    d
}

// Query implementations for HilbertRTreeLeg (merged from src/queries.rs)
//
// This section contains spatial query methods for the legacy HilbertRTreeLeg.
// It was previously kept in a separate `queries.rs` module and has been
// merged here so the legacy/reference implementation lives in a single file.

impl HilbertRTreeLeg {
    /// Internal helper for basic AABB intersection queries
    fn query_intersecting_internal(
        &self,
        query_min_x: f64,
        query_min_y: f64,
        query_max_x: f64,
        query_max_y: f64,
        limit: Option<usize>,
        results: &mut Vec<usize>,
    ) {
        let mut count = 0;
        for &idx in &self.sorted_order {
            if let Some(limit_val) = limit
                && count >= limit_val {
                    break;
                }
            
            let (min_x, min_y, max_x, max_y) = self.boxes[idx];
            
            // AABB intersection test
            if min_x <= query_max_x && max_x >= query_min_x &&
               min_y <= query_max_y && max_y >= query_min_y {
                results.push(idx);
                count += 1;
            }
        }
    }

    /// Internal helper for nearest queries (returns candidates with distances)
    fn query_nearest_internal(
        &self,
        x: f64,
        y: f64,
        k: Option<usize>,
    ) -> Vec<(f64, usize)> {
        if self.boxes.is_empty() {
            return Vec::new();
        }

        let mut distances: Vec<(f64, usize)> = Vec::with_capacity(self.boxes.len());
        
        for &idx in &self.sorted_order {
            let (min_x, min_y, max_x, max_y) = self.boxes[idx];
            let center_x = (min_x + max_x) / 2.0;
            let center_y = (min_y + max_y) / 2.0;
            
            let dx = center_x - x;
            let dy = center_y - y;
            let distance = (dx * dx + dy * dy).sqrt();
            
            distances.push((distance, idx));
        }

        // Sort by distance
        distances.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        
        if let Some(k_val) = k {
            distances.truncate(k_val);
        }
        
        distances
    }

    /// Internal helper for distance-based queries
    fn query_within_distance_internal(
        &self,
        x: f64,
        y: f64, 
        max_distance: f64,
        sort_by_distance: bool,
    ) -> Vec<(f64, usize)> {
        if max_distance < 0.0 || !max_distance.is_finite() {
            return Vec::new();
        }
        let max_distance_sq = max_distance * max_distance;
        let mut candidates: Vec<(f64, usize)> = Vec::new();

        for &idx in &self.sorted_order {
            let (min_x, min_y, max_x, max_y) = self.boxes[idx];
            
            // Calculate distance from point to box (0 if point is inside box)
            let dx = if x < min_x {
                min_x - x
            } else if x > max_x {
                x - max_x
            } else {
                0.0
            };

            let dy = if y < min_y {
                min_y - y
            } else if y > max_y {
                y - max_y
            } else {
                0.0
            };

            let distance_sq = dx * dx + dy * dy;
            
            if distance_sq <= max_distance_sq {
                let distance = if sort_by_distance { distance_sq.sqrt() } else { 0.0 };
                candidates.push((distance, idx));
            }
        }

        if sort_by_distance {
            candidates.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        }

        candidates
    }

    /// Internal helper for directional swept area queries
    fn query_in_direction_swept_internal(
        &self,
        rect_min_x: f64,
        rect_max_x: f64,
        rect_min_y: f64,
        rect_max_y: f64,
        direction_x: f64,
        direction_y: f64,
        distance: f64,
        calculate_distances: bool,
    ) -> Vec<(f64, usize)> {
        // Validate inputs
        if distance < 0.0 || !distance.is_finite() {
            return Vec::new();
        }
        
        // Normalize direction vector
        let dir_len_sq = direction_x * direction_x + direction_y * direction_y;
        if dir_len_sq <= 0.0 || !dir_len_sq.is_finite() {
            return Vec::new(); // Invalid direction vector
        }
        let dir_len = dir_len_sq.sqrt();
        let norm_dir_x = direction_x / dir_len;
        let norm_dir_y = direction_y / dir_len;
        
        // Calculate movement vector
        let movement_x = norm_dir_x * distance;
        let movement_y = norm_dir_y * distance;
        
        // Calculate the swept area bounds (union of start and end positions)
        let swept_min_x = (rect_min_x).min(rect_min_x + movement_x);
        let swept_max_x = (rect_max_x).max(rect_max_x + movement_x);
        let swept_min_y = (rect_min_y).min(rect_min_y + movement_y);
        let swept_max_y = (rect_max_y).max(rect_max_y + movement_y);

        // Calculate center of original rectangle for distance calculations
        let rect_center_x = if calculate_distances { (rect_min_x + rect_max_x) / 2.0 } else { 0.0 };
        let rect_center_y = if calculate_distances { (rect_min_y + rect_max_y) / 2.0 } else { 0.0 };

        let mut candidates: Vec<(f64, usize)> = Vec::new();

        // Find all boxes that intersect with the swept area
        for &idx in &self.sorted_order {
            let (min_x, min_y, max_x, max_y) = self.boxes[idx];
            
            // AABB intersection test with swept area
            if min_x <= swept_max_x && max_x >= swept_min_x &&
               min_y <= swept_max_y && max_y >= swept_min_y {
                
                let distance = if calculate_distances {
                    // Calculate distance from original rectangle center to box center
                    let center_x = (min_x + max_x) / 2.0;
                    let center_y = (min_y + max_y) / 2.0;
                    let dx = center_x - rect_center_x;
                    let dy = center_y - rect_center_y;
                    (dx * dx + dy * dy).sqrt()
                } else {
                    0.0
                };
                
                candidates.push((distance, idx));
            }
        }

        candidates
    }


    /// Queries for all boxes intersecting the given bounding box
    ///
    /// Results are appended to the output vector (not cleared first)
    ///
    /// # Arguments
    /// * `query_min_x`, `query_max_x`, `query_min_y`, `query_max_y` - Query rectangle bounds
    /// * `results` - Vector to append matching box indices to
    ///
    /// # Examples
    /// ```
    /// use aabb::HilbertRTreeLeg;
    /// 
    /// let mut tree = HilbertRTreeLeg::new();
    /// tree.add(0.0, 0.0, 2.0, 2.0);
    /// tree.add(1.0, 1.0, 3.0, 3.0);
    /// tree.build();
    /// 
    /// let mut results = Vec::new();
    /// tree.query_intersecting(1.5, 1.5, 2.5, 2.5, &mut results);
    /// assert_eq!(results.len(), 2);
    /// ```
    pub fn query_intersecting(
        &self,
        query_min_x: f64,
        query_min_y: f64,
        query_max_x: f64,
        query_max_y: f64,
        results: &mut Vec<usize>,
    ) {
        self.query_intersecting_internal(query_min_x, query_min_y, query_max_x, query_max_y, None, results);
    }

    /// Queries for the K first intersecting boxes with the given bounding box
    ///
    /// Stops after finding `k` results for performance. Results are appended 
    /// to the output vector (not cleared first).
    ///
    /// # Arguments
    /// * `query_min_x`, `query_max_x`, `query_min_y`, `query_max_y` - Query rectangle bounds
    /// * `k` - Maximum number of results to find
    /// * `results` - Vector to append matching box indices to
    ///
    /// # Examples
    /// ```
    /// use aabb::HilbertRTreeLeg;
    /// 
    /// let mut tree = HilbertRTreeLeg::new();
    /// tree.add(0.0, 2.0, 0.0, 2.0);
    /// tree.add(1.0, 3.0, 1.0, 3.0);
    /// tree.add(1.5, 2.5, 1.5, 2.5);
    /// tree.build();
    /// 
    /// let mut results = Vec::new();
    /// tree.query_intersecting_k(1.5, 1.5, 2.5, 2.5, 2, &mut results);
    /// assert!(results.len() <= 2);
    /// ```
    pub fn query_intersecting_k(
        &self,
        query_min_x: f64,
        query_min_y: f64,
        query_max_x: f64,
        query_max_y: f64,
        k: usize,
        results: &mut Vec<usize>,
    ) {
        self.query_intersecting_internal(query_min_x, query_min_y, query_max_x, query_max_y, Some(k), results);
    }

    /// Queries for all boxes that contain the given point
    ///
    /// Results are appended to the output vector (not cleared first)
    ///
    /// # Arguments
    /// * `x`, `y` - Point coordinates
    /// * `results` - Vector to append matching box indices to
    ///
    /// # Examples
    /// ```
    /// use aabb::HilbertRTreeLeg;
    /// 
    /// let mut tree = HilbertRTreeLeg::new();
    /// tree.add(0.0, 0.0, 2.0, 2.0);  // Contains (1.0, 1.0)
    /// tree.add(3.0, 3.0, 4.0, 4.0);  // Does not contain (1.0, 1.0)
    /// tree.build();
    /// 
    /// let mut results = Vec::new();
    /// tree.query_point(1.0, 1.0, &mut results);
    /// assert_eq!(results, vec![0]);
    /// ```
    pub fn query_point(&self, x: f64, y: f64, results: &mut Vec<usize>) {
        self.query_intersecting_internal(x, y, x, y, None, results);
    }

    /// Queries for all boxes that completely contain the given query rectangle
    ///
    /// A box contains the query rectangle if the query rectangle is entirely
    /// within the box's bounds.
    ///
    /// Results are appended to the output vector (not cleared first)
    ///
    /// # Arguments
    /// * `query_min_x`, `query_max_x`, `query_min_y`, `query_max_y` - Query rectangle bounds
    /// * `results` - Vector to append matching box indices to
    ///
    /// # Examples
    /// ```
    /// use aabb::HilbertRTreeLeg;
    /// 
    /// let mut tree = HilbertRTreeLeg::new();
    /// tree.add(0.0, 0.0, 4.0, 4.0);  // Large box - contains query
    /// tree.add(1.0, 1.0, 2.0, 2.0);  // Small box - does not contain query
    /// tree.build();
    /// 
    /// let mut results = Vec::new();
    /// tree.query_contain(1.5, 1.5, 3.5, 3.5, &mut results);
    /// assert_eq!(results, vec![0]);
    /// ```
    pub fn query_contain(
        &self,
        query_min_x: f64,
        query_min_y: f64,
        query_max_x: f64,
        query_max_y: f64,
        results: &mut Vec<usize>,
    ) {
        for &idx in &self.sorted_order {
            let (min_x, min_y, max_x, max_y) = self.boxes[idx];
            
            // Containment test: box contains query rectangle
            if min_x <= query_min_x && max_x >= query_max_x &&
               min_y <= query_min_y && max_y >= query_max_y {
                results.push(idx);
            }
        }
    }

    /// Queries for all boxes that are completely contained within the given query rectangle
    ///
    /// A box is contained by the query rectangle if the box is entirely
    /// within the query rectangle's bounds.
    ///
    /// Results are appended to the output vector (not cleared first)
    ///
    /// # Arguments
    /// * `query_min_x`, `query_max_x`, `query_min_y`, `query_max_y` - Query rectangle bounds
    /// * `results` - Vector to append matching box indices to
    ///
    /// # Examples
    /// ```
    /// use aabb::HilbertRTreeLeg;
    /// 
    /// let mut tree = HilbertRTreeLeg::new();
    /// tree.add(1.0, 2.0, 1.0, 2.0);  // Small box - contained by query
    /// tree.add(0.0, 4.0, 0.0, 4.0);  // Large box - not contained by query
    /// tree.build();
    /// 
    /// let mut results = Vec::new();
    /// tree.query_contained_within(0.5, 0.5, 3.5, 3.5, &mut results);
    /// assert_eq!(results, vec![0]);
    /// ```
    pub fn query_contained_within(
        &self,
        query_min_x: f64,
        query_min_y: f64,
        query_max_x: f64,
        query_max_y: f64,
        results: &mut Vec<usize>,
    ) {
        for &idx in &self.sorted_order {
            let (min_x, min_y, max_x, max_y) = self.boxes[idx];
            
            // Containment test: query rectangle contains box
            if query_min_x <= min_x && query_max_x >= max_x &&
               query_min_y <= min_y && query_max_y >= max_y {
                results.push(idx);
            }
        }
    }

    /// Queries for the K nearest boxes to a point (by distance to box center)
    ///
    /// Results are appended to the output vector (not cleared first).
    /// Results are sorted by distance (closest first).
    ///
    /// # Arguments
    /// * `x`, `y` - Query point coordinates
    /// * `k` - Maximum number of nearest boxes to find
    /// * `results` - Vector to append matching box indices to
    ///
    /// # Examples
    /// ```
    /// use aabb::HilbertRTreeLeg;
    /// 
    /// let mut tree = HilbertRTreeLeg::new();
    /// tree.add(0.0, 1.0, 0.0, 1.0);   // Center at (0.5, 0.5)
    /// tree.add(2.0, 3.0, 2.0, 3.0);   // Center at (2.5, 2.5)
    /// tree.add(4.0, 5.0, 4.0, 5.0);   // Center at (4.5, 4.5)
    /// tree.build();
    /// 
    /// let mut results = Vec::new();
    /// tree.query_nearest_k(1.0, 1.0, 2, &mut results);
    /// assert!(results.len() <= 2);
    /// // Results are sorted by distance, so first should be closest
    /// ```
    pub fn query_nearest_k(&self, x: f64, y: f64, k: usize, results: &mut Vec<usize>) {
        if k == 0 {
            return;
        }

        let candidates = self.query_nearest_internal(x, y, Some(k));
        for (_, idx) in candidates {
            results.push(idx);
        }
    }

    /// Queries for the nearest box to a point (by distance to box center)
    ///
    /// Returns the index of the nearest box, or None if the tree is empty.
    ///
    /// # Arguments
    /// * `x`, `y` - Query point coordinates
    ///
    /// # Examples
    /// ```
    /// use aabb::HilbertRTreeLeg;
    /// 
    /// let mut tree = HilbertRTreeLeg::new();
    /// tree.add(0.0, 1.0, 0.0, 1.0);   // Center at (0.5, 0.5)
    /// tree.add(2.0, 3.0, 2.0, 3.0);   // Center at (2.5, 2.5)
    /// tree.build();
    /// 
    /// let nearest = tree.query_nearest(0.6, 0.6);
    /// assert_eq!(nearest, Some(0)); // First box is closer
    /// ```
    pub fn query_nearest(&self, x: f64, y: f64) -> Option<usize> {
        let candidates = self.query_nearest_internal(x, y, Some(1));
        candidates.first().map(|(_, idx)| *idx)
    }

    /// Queries for all boxes within a certain distance of a point
    ///
    /// Distance is measured from the point to the closest edge of each box.
    /// Results are appended to the output vector (not cleared first).
    ///
    /// # Arguments
    /// * `x`, `y` - Query point coordinates
    /// * `max_distance` - Maximum distance from point to box edge
    /// * `results` - Vector to append matching box indices to
    ///
    /// # Examples
    /// ```
    /// use aabb::HilbertRTreeLeg;
    /// 
    /// let mut tree = HilbertRTreeLeg::new();
    /// tree.add(0.0, 1.0, 0.0, 1.0);
    /// tree.add(3.0, 4.0, 3.0, 4.0);   // Far away
    /// tree.build();
    /// 
    /// let mut results = Vec::new();
    /// tree.query_circle(1.5, 1.5, 1.0, &mut results);
    /// // Should find first box (distance ~0.7) but not second box
    /// ```
    pub fn query_within_distance(&self, x: f64, y: f64, max_distance: f64, results: &mut Vec<usize>) {
        let candidates = self.query_within_distance_internal(x, y, max_distance, false);
        for (_, idx) in candidates {
            results.push(idx);
        }
    }

    /// Queries for all boxes within a circular region
    ///
    /// Finds boxes that intersect with a circle centered at the given point.
    /// Results are appended to the output vector (not cleared first).
    ///
    /// # Arguments
    /// * `center_x`, `center_y` - Circle center coordinates
    /// * `radius` - Circle radius
    /// * `results` - Vector to append matching box indices to
    ///
    /// # Examples
    /// ```
    /// use aabb::HilbertRTreeLeg;
    /// 
    /// let mut tree = HilbertRTreeLeg::new();
    /// tree.add(0.0, 1.0, 0.0, 1.0);   // Intersects circle
    /// tree.add(5.0, 6.0, 5.0, 6.0);   // Outside circle
    /// tree.build();
    /// 
    /// let mut results = Vec::new();
    /// tree.query_circle(1.5, 1.5, 2.0, &mut results);
    /// // Should find first box but not second
    /// ```
    pub fn query_circle(&self, center_x: f64, center_y: f64, radius: f64, results: &mut Vec<usize>) {
        if radius < 0.0 || !radius.is_finite() {
            return;
        }
        let radius_sq = radius * radius;

        for &idx in &self.sorted_order {
            let (min_x, min_y, max_x, max_y) = self.boxes[idx];
            
            // Find closest point on box to circle center
            let closest_x = center_x.clamp(min_x, max_x);
            let closest_y = center_y.clamp(min_y, max_y);
            
            // Check if closest point is within circle
            let dx = closest_x - center_x;
            let dy = closest_y - center_y;
            let distance_sq = dx * dx + dy * dy;
            
            if distance_sq <= radius_sq {
                results.push(idx);
            }
        }
    }



    /// Queries for the K nearest boxes that intersect with a rectangle's movement path
    ///
    /// Takes a starting rectangle and finds all boxes that intersect as the rectangle
    /// moves in the specified direction from position 0 to the given distance.
    /// Returns the K nearest boxes (by distance to original rectangle center), sorted by distance.
    ///
    /// Results are appended to the output vector (not cleared first).
    ///
    /// # Arguments
    /// * `rect_min_x`, `rect_max_x`, `rect_min_y`, `rect_max_y` - Starting rectangle bounds
    /// * `direction_x`, `direction_y` - Direction vector (does not need to be normalized)
    /// * `k` - Maximum number of boxes to find  
    /// * `distance` - How far to move the rectangle in the given direction
    /// * `results` - Vector to append matching box indices to
    ///
    /// # Examples
    /// ```
    /// use aabb::HilbertRTreeLeg;
    /// 
    /// let mut tree = HilbertRTreeLeg::new();
    /// tree.add(2.0, 3.0, 0.0, 1.0);   // To the right, close
    /// tree.add(4.0, 5.0, 0.0, 1.0);   // To the right, far
    /// tree.add(0.0, 1.0, 2.0, 3.0);   // Above
    /// tree.build();
    /// 
    /// let mut results = Vec::new();
    /// // Start with rectangle (1,1) to (1.5,1.5) and move right, find 1 closest
    /// tree.query_in_direction_k(1.0, 1.5, 1.0, 1.5, 1.0, 0.0, 1, 10.0, &mut results);
    /// // Should find closest box that intersects the movement path
    /// ```
    pub fn query_in_direction_k(
        &self,
        rect_min_x: f64,
        rect_min_y: f64,
        rect_max_x: f64,
        rect_max_y: f64,
        direction_x: f64,
        direction_y: f64,
        k: usize,
        distance: f64,
        results: &mut Vec<usize>,
    ) {
        if k == 0 {
            return;
        }

        let candidates = self.query_in_direction_swept_internal(
            rect_min_x, rect_max_x, rect_min_y, rect_max_y,
            direction_x, direction_y, distance, true
        );
        
        // Sort by distance and take first k
        let mut sorted_candidates = candidates;
        sorted_candidates.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        
        for (_, idx) in sorted_candidates.into_iter().take(k) {
            results.push(idx);
        }
    }

    /// Queries for boxes that intersect with a rectangle moving in a specific direction
    ///
    /// Takes a starting rectangle and finds all boxes that intersect as the rectangle
    /// moves in the specified direction from position 0 to the given distance.
    /// This creates a swept area that covers the entire movement path.
    ///
    /// Results are appended to the output vector (not cleared first).
    ///
    /// # Arguments
    /// * `rect_min_x`, `rect_max_x`, `rect_min_y`, `rect_max_y` - Starting rectangle bounds
    /// * `direction_x`, `direction_y` - Direction vector (does not need to be normalized)
    /// * `distance` - How far to move the rectangle in the given direction
    /// * `results` - Vector to append matching box indices to
    ///
    /// # Examples
    /// ```
    /// use aabb::HilbertRTreeLeg;
    /// 
    /// let mut tree = HilbertRTreeLeg::new();
    /// tree.add(2.0, 3.0, 0.0, 1.0);   // To the right
    /// tree.add(0.0, 1.0, 2.0, 3.0);   // Above
    /// tree.build();
    /// 
    /// let mut results = Vec::new();
    /// // Start with rectangle (1,1) to (1.5,1.5) and move right by distance 5
    /// tree.query_in_direction(1.0, 1.0, 1.5, 1.5, 1.0, 0.0, 5.0, &mut results);
    /// // Finds all boxes intersected as rectangle moves from (1,1)-(1.5,1.5) to (6,1)-(6.5,1.5)
    /// ```
    pub fn query_in_direction(
        &self,
        rect_min_x: f64,
        rect_min_y: f64,
        rect_max_x: f64,
        rect_max_y: f64,
        direction_x: f64,
        direction_y: f64,
        distance: f64,
        results: &mut Vec<usize>,
    ) {
        let candidates = self.query_in_direction_swept_internal(
            rect_min_x, rect_max_x, rect_min_y, rect_max_y,
            direction_x, direction_y, distance, false
        );
        
        for (_, idx) in candidates {
            results.push(idx);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hilbert_rtree_new() {
        let tree = HilbertRTreeLeg::new();
        assert!(tree.is_empty());
        assert_eq!(tree.len(), 0);
    }

    #[test]
    fn test_hilbert_rtree_add() {
        let mut tree = HilbertRTreeLeg::new();
        tree.add(0.0, 0.0, 1.0, 1.0);
        tree.add(2.0, 2.0, 3.0, 3.0);
        
        assert_eq!(tree.len(), 2);
        assert!(!tree.is_empty());
    }

    #[test]
    fn test_hilbert_rtree_build() {
        let mut tree = HilbertRTreeLeg::new();
        tree.add(0.0, 0.0, 1.0, 1.0);
        tree.add(2.0, 2.0, 3.0, 3.0);
        tree.build();
        
        assert!(tree.built);
    }

    #[test]
    fn test_hilbert_rtree_build_empty() {
        let mut tree = HilbertRTreeLeg::new();
        tree.build();  // Build empty tree
        
        assert!(tree.is_empty());
        assert_eq!(tree.len(), 0);
    }

    #[test]
    fn test_hilbert_rtree_with_capacity() {
        let tree = HilbertRTreeLeg::with_capacity(100);
        assert!(tree.is_empty());
        assert_eq!(tree.len(), 0);
    }

    #[test]
    fn test_hilbert_rtree_default() {
        let tree = HilbertRTreeLeg::default();
        assert!(tree.is_empty());
        assert_eq!(tree.len(), 0);
    }

    #[test]
    fn test_hilbert_rtree_extreme_coordinates() {
        let mut tree = HilbertRTreeLeg::new();
        
        // Test with very large coordinates
        tree.add(1e10, 1e10, 1e10 + 1.0, 1e10 + 1.0);
        
        // Test with very small coordinates
        tree.add(1e-10, 1e-10, 2e-10, 2e-10);
        
        // Test with mixed positive/negative
        tree.add(-1000.0, -1000.0, -999.0, -999.0);
        
        tree.build();
        
        assert_eq!(tree.len(), 3);
        assert!(tree.built);
    }

    #[test]
    fn test_hilbert_rtree_infinite_coordinates() {
        let mut tree = HilbertRTreeLeg::new();
        
        // Test with infinite coordinates - should be handled safely
        tree.add(f64::INFINITY, 0.0, f64::INFINITY, 1.0);
        tree.add(f64::NEG_INFINITY, 0.0, f64::NEG_INFINITY, 1.0);
        tree.add(f64::NAN, 0.0, f64::NAN, 1.0);
        
        tree.build();  // Should not panic
        
        assert_eq!(tree.len(), 3);
        assert!(tree.built);
    }

    #[test]
    fn test_hilbert_rtree_rebuild() {
        let mut tree = HilbertRTreeLeg::new();
        tree.add(0.0, 0.0, 1.0, 1.0);
        tree.build();
        
        assert!(tree.built);
        
        // Add more boxes - should mark as not built
        tree.add(2.0, 2.0, 3.0, 3.0);
        assert!(!tree.built);
        
        // Rebuild
        tree.build();
        assert!(tree.built);
        assert_eq!(tree.len(), 2);
    }

    #[test]
    fn test_hilbert_rtree_clone() {
        let mut tree = HilbertRTreeLeg::new();
        tree.add(0.0, 0.0, 1.0, 1.0);
        tree.add(2.0, 2.0, 3.0, 3.0);
        tree.build();
        
        let cloned = tree.clone();
        
        assert_eq!(tree.len(), cloned.len());
        assert_eq!(tree.built, cloned.built);
        assert_eq!(tree.is_empty(), cloned.is_empty());
    }

}