//! Query implementations for HilbertRTreeLeg (legacy/reference implementation)
//!
//! This module contains all spatial query methods for the legacy HilbertRTreeLeg.
//! 
//! # Note
//! This is a reference implementation. For production use, see [`HilbertRTree`].

#![doc(hidden)]

use crate::HilbertRTreeLeg;

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
            if let Some(limit_val) = limit {
                if count >= limit_val {
                    break;
                }
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
    /// use aabb::HilbertRTree;
    /// 
    /// let mut tree = HilbertRTree::new();
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
    /// use aabb::HilbertRTree;
    /// 
    /// let mut tree = HilbertRTree::new();
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
    /// use aabb::HilbertRTree;
    /// 
    /// let mut tree = HilbertRTree::new();
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
    /// use aabb::HilbertRTree;
    /// 
    /// let mut tree = HilbertRTree::new();
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
    /// use aabb::HilbertRTree;
    /// 
    /// let mut tree = HilbertRTree::new();
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
    /// use aabb::HilbertRTree;
    /// 
    /// let mut tree = HilbertRTree::new();
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
    /// use aabb::HilbertRTree;
    /// 
    /// let mut tree = HilbertRTree::new();
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
    /// use aabb::HilbertRTree;
    /// 
    /// let mut tree = HilbertRTree::new();
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
    /// use aabb::HilbertRTree;
    /// 
    /// let mut tree = HilbertRTree::new();
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
    /// use aabb::HilbertRTree;
    /// 
    /// let mut tree = HilbertRTree::new();
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
    /// use aabb::HilbertRTree;
    /// 
    /// let mut tree = HilbertRTree::new();
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
    use crate::HilbertRTreeLeg;

    #[test]
    fn test_hilbert_rtree_query_intersecting() {
        let mut tree = HilbertRTreeLeg::new();
        tree.add(0.0, 0.0, 1.0, 1.0);
        tree.add(2.0, 2.0, 3.0, 3.0);
        tree.add(0.5, 0.5, 1.5, 1.5);
        tree.build();
        
        let mut results = Vec::new();
        tree.query_intersecting(0.7, 0.7, 1.3, 1.3, &mut results);
        
        // Should find boxes 0 and 2 (both intersect the query)
        assert_eq!(results.len(), 2);
        assert!(results.contains(&0));
        assert!(results.contains(&2));
    }

    #[test]
    fn test_hilbert_rtree_query_no_intersections() {
        let mut tree = HilbertRTreeLeg::new();
        tree.add(0.0, 1.0, 0.0, 1.0);
        tree.add(5.0, 6.0, 5.0, 6.0);
        tree.build();
        
        let mut results = Vec::new();
        tree.query_intersecting(2.0, 3.0, 2.0, 3.0, &mut results);
        
        // No boxes intersect the query
        assert_eq!(results.len(), 0);
    }

    #[test]
    fn test_hilbert_rtree_query_point() {
        let mut tree = HilbertRTreeLeg::new();
        tree.add(0.0, 0.0, 2.0, 2.0);  // Contains (1.0, 1.0)
        tree.add(3.0, 3.0, 4.0, 4.0);  // Does not contain (1.0, 1.0)
        tree.add(0.5, 0.5, 1.5, 1.5);  // Also contains (1.0, 1.0)
        tree.build();
        
        let mut results = Vec::new();
        tree.query_point(1.0, 1.0, &mut results);
        
        // Should find boxes 0 and 2
        assert_eq!(results.len(), 2);
        assert!(results.contains(&0));
        assert!(results.contains(&2));
    }

    #[test]
    fn test_hilbert_rtree_query_containing() {
        let mut tree = HilbertRTreeLeg::new();
        // add(min_x, min_y, max_x, max_y)
        tree.add(0.0, 0.0, 4.0, 4.0);  // Large box - contains query
        tree.add(1.0, 2.0, 1.5, 2.5);  // Small box - does not contain query
        tree.add(-1.0, -1.0, 5.0, 5.0); // Even larger box - also contains query
        tree.build();
        
        let mut results = Vec::new();
        tree.query_contain(1.5, 1.5, 3.5, 3.5, &mut results);
        
        // Should find boxes 0 and 2 (both contain the query rectangle)
        assert_eq!(results.len(), 2);
        assert!(results.contains(&0));
        assert!(results.contains(&2));
    }

    #[test]
    fn test_hilbert_rtree_query_contained_by() {
        let mut tree = HilbertRTreeLeg::new();
        // add(min_x, min_y, max_x, max_y)
        tree.add(1.0, 1.0, 2.0, 2.0);  // Small box - contained by query
        tree.add(0.0, 0.0, 4.0, 4.0);  // Large box - not contained by query
        tree.add(2.0, 2.0, 3.0, 3.0);  // Another small box - also contained
        tree.build();
        
        let mut results = Vec::new();
        tree.query_contained_within(0.5, 0.5, 3.5, 3.5, &mut results);
        
        // Should find boxes 0 and 2 (both are contained by the query rectangle)
        assert_eq!(results.len(), 2);
        assert!(results.contains(&0));
        assert!(results.contains(&2));
    }

    #[test]
    fn test_hilbert_rtree_query_intersecting_k() {
        let mut tree = HilbertRTreeLeg::new();
        // add(min_x, min_y, max_x, max_y)
        tree.add(0.0, 0.0, 2.0, 2.0);
        tree.add(1.0, 1.0, 3.0, 3.0);
        tree.add(1.5, 1.5, 2.5, 2.5);
        tree.add(1.2, 1.2, 2.2, 2.2);
        tree.build();
        
        let mut results = Vec::new();
        tree.query_intersecting_k(1.5, 1.5, 2.5, 2.5, 2, &mut results);
        
        // Should find at most 2 results due to k limit
        assert!(results.len() <= 2);
        assert!(results.len() > 0); // Should find at least some
    }

    #[test]
    fn test_hilbert_rtree_query_nearest_k() {
        let mut tree = HilbertRTreeLeg::new();
        tree.add(0.0, 1.0, 0.0, 1.0);   // Center at (0.5, 0.5)
        tree.add(2.0, 3.0, 2.0, 3.0);   // Center at (2.5, 2.5)
        tree.add(4.0, 5.0, 4.0, 5.0);   // Center at (4.5, 4.5)
        tree.build();
        
        let mut results = Vec::new();
        tree.query_nearest_k(1.0, 1.0, 2, &mut results);
        
        assert_eq!(results.len(), 2);
        // First result should be closest (box 0)
        assert_eq!(results[0], 0);
        // Second result should be next closest (box 1)
        assert_eq!(results[1], 1);
    }

    #[test]
    fn test_hilbert_rtree_query_nearest() {
        let mut tree = HilbertRTreeLeg::new();
        tree.add(0.0, 1.0, 0.0, 1.0);   // Center at (0.5, 0.5)
        tree.add(2.0, 3.0, 2.0, 3.0);   // Center at (2.5, 2.5)
        tree.build();
        
        let nearest = tree.query_nearest(0.6, 0.6);
        assert_eq!(nearest, Some(0)); // First box is closer
        
        let nearest = tree.query_nearest(2.4, 2.4);
        assert_eq!(nearest, Some(1)); // Second box is closer
    }

    #[test]
    fn test_hilbert_rtree_query_within_distance() {
        let mut tree = HilbertRTreeLeg::new();
        tree.add(0.0, 0.0, 1.0, 1.0);   // Close
        tree.add(5.0, 5.0, 6.0, 6.0);   // Far away
        tree.build();
        
        let mut results = Vec::new();
        tree.query_within_distance(1.5, 1.5, 1.0, &mut results);
        
        // Should find first box (distance ~0.7) but not second box
        assert_eq!(results.len(), 1);
        assert_eq!(results[0], 0);
    }

    #[test]
    fn test_hilbert_rtree_query_circle() {
        let mut tree = HilbertRTreeLeg::new();
        tree.add(0.0, 1.0, 0.0, 1.0);   // Intersects circle
        tree.add(5.0, 6.0, 5.0, 6.0);   // Outside circle
        tree.build();
        
        let mut results = Vec::new();
        tree.query_circle(1.5, 1.5, 2.0, &mut results);
        
        // Should find first box but not second
        assert_eq!(results.len(), 1);
        assert_eq!(results[0], 0);
    }

    #[test]
    fn test_hilbert_rtree_query_in_direction() {
        let mut tree = HilbertRTreeLeg::new();
        // add(min_x, min_y, max_x, max_y)
        tree.add(2.0, 0.0, 3.0, 1.0);   // To the right, center at (2.5, 0.5)
        tree.add(0.0, 2.0, 1.0, 3.0);   // Above, center at (0.5, 2.5)
        tree.build();
        
        let mut results = Vec::new();
        // Start with small rectangle at (1,0.5) to (1.1,0.6) and move right
        tree.query_in_direction(1.0, 0.5, 1.1, 0.6, 1.0, 0.0, 5.0, &mut results);
        
        // Should find first box (to the right) but not second box (above)
        assert_eq!(results.len(), 1);
        assert_eq!(results[0], 0);
    }

    #[test]
    fn test_hilbert_rtree_query_in_direction_k() {
        let mut tree = HilbertRTreeLeg::new();
        // add(min_x, min_y, max_x, max_y)
        tree.add(2.0, 0.0, 3.0, 1.0);   // To the right, close (center at 2.5, 0.5)
        tree.add(4.0, 0.0, 5.0, 1.0);   // To the right, far (center at 4.5, 0.5)
        tree.add(0.0, 2.0, 1.0, 3.0);   // Above (center at 0.5, 2.5)
        tree.build();
        
        let mut results = Vec::new();
        // Start with small rectangle at (1,0.5) to (1.1,0.6) (center at 1.05, 0.55)
        // Move right by 10.0 to create swept area, find 2 closest
        tree.query_in_direction_k(1.0, 0.5, 1.1, 0.6, 1.0, 0.0, 2, 10.0, &mut results);
        
        // Should find both boxes to the right, closest first (sorted by distance from original rectangle center)
        assert_eq!(results.len(), 2);
        assert_eq!(results[0], 0); // Closest (distance ~1.45)
        assert_eq!(results[1], 1); // Further (distance ~3.45)
    }

    #[test]
    fn test_hilbert_rtree_query_in_direction_swept_area() {
        let mut tree = HilbertRTreeLeg::new();
        // add(min_x, min_y, max_x, max_y)
        tree.add(2.0, 0.0, 3.0, 1.0);   // To the right, close
        tree.add(6.0, 0.0, 7.0, 1.0);   // To the right, far
        tree.add(0.0, 2.0, 1.0, 3.0);   // Above
        tree.build();
        
        let mut results = Vec::new();
        // Start with small rectangle at (1,0.5) to (1.1,0.6) and move right by 8.0
        // This creates swept area from (1,0.5)-(1.1,0.6) to (9,0.5)-(9.1,0.6)
        tree.query_in_direction(1.0, 0.5, 1.1, 0.6, 1.0, 0.0, 8.0, &mut results);
        
        // Should find both boxes to the right (both intersect the swept area)
        assert_eq!(results.len(), 2);
        assert!(results.contains(&0)); // Close box
        assert!(results.contains(&1)); // Far box
        assert!(!results.contains(&2)); // Above box should not intersect
    }

    #[test]
    fn test_hilbert_rtree_empty_queries() {
        let tree = HilbertRTreeLeg::new();
        
        // Test all queries on empty tree
        let mut results = Vec::new();
        tree.query_intersecting(0.0, 1.0, 0.0, 1.0, &mut results);
        assert_eq!(results.len(), 0);
        
        tree.query_point(0.0, 0.0, &mut results);
        assert_eq!(results.len(), 0);
        
        tree.query_nearest_k(0.0, 0.0, 5, &mut results);
        assert_eq!(results.len(), 0);
        
        assert_eq!(tree.query_nearest(0.0, 0.0), None);
        
        tree.query_within_distance(0.0, 0.0, 1.0, &mut results);
        assert_eq!(results.len(), 0);
        
        tree.query_circle(0.0, 0.0, 1.0, &mut results);
        assert_eq!(results.len(), 0);
        
        tree.query_in_direction(0.0, 0.1, 0.0, 0.1, 1.0, 0.0, 1.0, &mut results);
        assert_eq!(results.len(), 0);
        
        tree.query_in_direction_k(0.0, 0.1, 0.0, 0.1, 1.0, 0.0, 5, 1.0, &mut results);
        assert_eq!(results.len(), 0);
        
        tree.query_in_direction(0.0, 0.1, 0.0, 0.1, 1.0, 0.0, 1.0, &mut results);
        assert_eq!(results.len(), 0);
    }

    #[test]
    fn test_floating_point_edge_cases() {
        let mut tree = HilbertRTreeLeg::new();
        tree.add(0.0, 1.0, 0.0, 1.0);
        tree.build();
        
        let mut results = Vec::new();
        
        // Test negative distances
        tree.query_within_distance(0.5, 0.5, -1.0, &mut results);
        assert_eq!(results.len(), 0, "Negative distance should return no results");
        
        // Test infinite distances
        results.clear();
        tree.query_within_distance(0.5, 0.5, f64::INFINITY, &mut results);
        assert_eq!(results.len(), 0, "Infinite distance should return no results");
        
        // Test NaN distances
        results.clear();
        tree.query_within_distance(0.5, 0.5, f64::NAN, &mut results);
        assert_eq!(results.len(), 0, "NaN distance should return no results");
        
        // Test negative radius for circle queries
        results.clear();
        tree.query_circle(0.5, 0.5, -1.0, &mut results);
        assert_eq!(results.len(), 0, "Negative radius should return no results");
        
        // Test infinite radius for circle queries
        results.clear();
        tree.query_circle(0.5, 0.5, f64::INFINITY, &mut results);
        assert_eq!(results.len(), 0, "Infinite radius should return no results");
        
        // Test NaN radius for circle queries
        results.clear();
        tree.query_circle(0.5, 0.5, f64::NAN, &mut results);
        assert_eq!(results.len(), 0, "NaN radius should return no results");
    }

    #[test]
    fn test_directional_query_edge_cases() {
        let mut tree = HilbertRTreeLeg::new();
        tree.add(1.0, 2.0, 1.0, 2.0);
        tree.build();
        
        let mut results = Vec::new();
        
        // Test zero direction vector
        tree.query_in_direction(0.0, 0.1, 0.0, 0.1, 0.0, 0.0, 1.0, &mut results);
        assert_eq!(results.len(), 0, "Zero direction vector should return no results");
        
        // Test infinite direction vector
        results.clear();
        tree.query_in_direction(0.0, 0.1, 0.0, 0.1, f64::INFINITY, 1.0, 1.0, &mut results);
        assert_eq!(results.len(), 0, "Infinite direction vector should return no results");
        
        // Test NaN direction vector
        results.clear();
        tree.query_in_direction(0.0, 0.1, 0.0, 0.1, f64::NAN, 1.0, 1.0, &mut results);
        assert_eq!(results.len(), 0, "NaN direction vector should return no results");
        
        // Test negative distance
        results.clear();
        tree.query_in_direction(0.0, 0.1, 0.0, 0.1, 1.0, 0.0, -1.0, &mut results);
        assert_eq!(results.len(), 0, "Negative distance should return no results");
        
        // Test infinite distance
        results.clear();
        tree.query_in_direction(0.0, 0.1, 0.0, 0.1, 1.0, 0.0, f64::INFINITY, &mut results);
        assert_eq!(results.len(), 0, "Infinite distance should return no results");
        
        // Test NaN distance
        results.clear();
        tree.query_in_direction(0.0, 0.1, 0.0, 0.1, 1.0, 0.0, f64::NAN, &mut results);
        assert_eq!(results.len(), 0, "NaN distance should return no results");
        
        // Test directional k-query with k=0
        results.clear();
        tree.query_in_direction_k(0.0, 0.1, 0.0, 0.1, 1.0, 0.0, 0, 1.0, &mut results);
        assert_eq!(results.len(), 0, "k=0 should return no results");
        
        // Test nearest k-query with k=0
        results.clear();
        tree.query_nearest_k(0.5, 0.5, 0, &mut results);
        assert_eq!(results.len(), 0, "k=0 should return no results");
    }

    #[test]
    fn test_query_intersecting_k_edge_cases() {
        let mut tree = HilbertRTreeLeg::new();
        tree.add(0.0, 0.0, 1.0, 1.0);   // Box from (0,0) to (1,1)
        tree.add(0.5, 0.5, 1.5, 1.5);   // Box from (0.5,0.5) to (1.5,1.5)
        tree.add(0.2, 0.2, 1.2, 1.2);   // Box from (0.2,0.2) to (1.2,1.2)
        tree.build();
        
        let mut results = Vec::new();
        
        // Test k=0
        tree.query_intersecting_k(0.5, 0.5, 1.0, 1.0, 0, &mut results);
        assert_eq!(results.len(), 0, "k=0 should return no results");
        
        // Test k=1 when multiple boxes intersect
        results.clear();
        tree.query_intersecting_k(0.5, 0.5, 1.0, 1.0, 1, &mut results);
        assert_eq!(results.len(), 1, "k=1 should return at most 1 result");
        
        // Test k larger than available results
        results.clear();
        tree.query_intersecting_k(0.5, 0.5, 1.0, 1.0, 10, &mut results);
        assert!(results.len() <= 10 && results.len() > 0, "Should return all available results up to k");
    }

    #[test]
    fn test_point_query_boundary_cases() {
        let mut tree = HilbertRTreeLeg::new();
        // add(min_x, min_y, max_x, max_y)
        tree.add(0.0, 0.0, 1.0, 1.0);  // Box from (0,0) to (1,1)
        tree.build();
        
        let mut results = Vec::new();
        
        // Test point exactly on boundaries
        tree.query_point(0.0, 0.0, &mut results);  // Bottom-left corner
        assert_eq!(results.len(), 1, "Point on boundary should be contained");
        
        results.clear();
        tree.query_point(1.0, 1.0, &mut results);  // Top-right corner
        assert_eq!(results.len(), 1, "Point on boundary should be contained");
        
        results.clear();
        tree.query_point(0.5, 0.0, &mut results);  // Bottom edge
        assert_eq!(results.len(), 1, "Point on edge should be contained");
        
        results.clear();
        tree.query_point(1.0, 0.5, &mut results);  // Right edge
        assert_eq!(results.len(), 1, "Point on edge should be contained");
        
        // Test point just outside boundaries
        results.clear();
        tree.query_point(-0.001, 0.5, &mut results);
        assert_eq!(results.len(), 0, "Point just outside should not be contained");
        
        results.clear();
        tree.query_point(1.001, 0.5, &mut results);
        assert_eq!(results.len(), 0, "Point just outside should not be contained");
    }

    #[test]
    fn test_distance_query_precision() {
        let mut tree = HilbertRTreeLeg::new();
        // add(min_x, min_y, max_x, max_y)
        tree.add(0.0, 0.0, 1.0, 1.0);  // Box centered at (0.5, 0.5)
        tree.build();
        
        let mut results = Vec::new();
        
        // Test distance to box edge (point outside box)
        tree.query_within_distance(2.0, 0.5, 1.0, &mut results);  // Distance = 1.0 to edge
        assert_eq!(results.len(), 1, "Should find box within distance to edge");
        
        results.clear();
        tree.query_within_distance(2.0, 0.5, 0.999, &mut results);  // Just under distance
        assert_eq!(results.len(), 0, "Should not find box when just outside distance");
        
        // Test distance when point is inside box (distance = 0)
        results.clear();
        tree.query_within_distance(0.5, 0.5, 0.001, &mut results);  // Point inside box
        assert_eq!(results.len(), 1, "Should find box when point is inside (distance=0)");
    }

    #[test]
    fn test_circle_query_precision() {
        let mut tree = HilbertRTreeLeg::new();
        // add(min_x, min_y, max_x, max_y)
        tree.add(2.0, 2.0, 3.0, 3.0);  // Box from (2,2) to (3,3)
        tree.build();
        
        let mut results = Vec::new();
        
        // Test circle that just touches the box
        tree.query_circle(1.0, 2.5, 1.0, &mut results);  // Circle center at (1, 2.5), radius 1.0
        assert_eq!(results.len(), 1, "Circle just touching box should intersect");
        
        results.clear();
        tree.query_circle(1.0, 2.5, 0.999, &mut results);  // Just under touching distance
        assert_eq!(results.len(), 0, "Circle just short of touching should not intersect");
        
        // Test circle completely containing the box
        results.clear();
        tree.query_circle(2.5, 2.5, 2.0, &mut results);  // Large circle centered on box
        assert_eq!(results.len(), 1, "Circle containing box should intersect");
    }

    #[test]
    fn test_nearest_query_ordering() {
        let mut tree = HilbertRTreeLeg::new();
        tree.add(0.0, 1.0, 0.0, 1.0);   // Center at (0.5, 0.5) - closest
        tree.add(3.0, 4.0, 3.0, 4.0);   // Center at (3.5, 3.5) - farthest  
        tree.add(1.5, 2.5, 1.5, 2.5);   // Center at (2.0, 2.0) - middle
        tree.build();
        
        // Query from point (1.0, 1.0)
        let mut results = Vec::new();
        tree.query_nearest_k(1.0, 1.0, 3, &mut results);
        
        assert_eq!(results.len(), 3);
        assert_eq!(results[0], 0, "Closest box should be first");
        assert_eq!(results[1], 2, "Middle distance box should be second");
        assert_eq!(results[2], 1, "Farthest box should be last");
        
        // Test single nearest
        let nearest = tree.query_nearest(1.0, 1.0);
        assert_eq!(nearest, Some(0), "Single nearest should return closest box");
    }

    #[test]
    fn test_complex_directional_queries() {
        let mut tree = HilbertRTreeLeg::new();
        // add(min_x, min_y, max_x, max_y)
        // Create a grid of boxes
        tree.add(0.0, 0.0, 1.0, 1.0);   // Bottom-left
        tree.add(2.0, 0.0, 3.0, 1.0);   // Bottom-right
        tree.add(0.0, 2.0, 1.0, 3.0);   // Top-left
        tree.add(2.0, 2.0, 3.0, 3.0);   // Top-right
        tree.build();
        
        let mut results = Vec::new();
        
        // Query extending right from center-left
        tree.query_in_direction(1.0, 1.0, 0.4, 0.6, 1.0, 0.0, 2.0, &mut results);
        assert!(results.contains(&1), "Should find box to the right");
        assert!(!results.contains(&2), "Should not find box above");
        
        results.clear();
        
        // Query extending up from center-bottom
        tree.query_in_direction(0.5, 0.6, 1.0, 1.1, 0.0, 1.0, 2.0, &mut results);
        assert!(results.contains(&2), "Should find box above");
        assert!(!results.contains(&1), "Should not find box to the right");
        
        results.clear();
        
        // Test diagonal direction
        tree.query_in_direction(0.4, 0.6, 0.4, 0.6, 1.0, 1.0, 3.0, &mut results);
        assert!(results.contains(&3), "Should find box diagonally");
    }

    #[test]
    fn test_directional_swept_area() {
        let mut tree = HilbertRTreeLeg::new();
        // add(min_x, min_y, max_x, max_y)
        tree.add(2.0, 0.0, 3.0, 1.0);   // Close box
        tree.add(4.0, 0.0, 5.0, 1.0);   // Far box  
        tree.add(3.0, 0.0, 4.0, 1.0);   // Middle box
        tree.build();
        
        let mut results = Vec::new();
        
        // Query with short movement - should only intersect close box
        tree.query_in_direction(1.0, 0.4, 1.1, 0.6, 1.0, 0.0, 1.5, &mut results);
        
        // Swept area from (1,0.4)-(1.1,0.6) to (2.5,0.4)-(2.6,0.6) should intersect close box
        assert_eq!(results.len(), 1);
        assert!(results.contains(&0), "Should contain close box");
        
        results.clear();
        
        // Query with longer movement - should intersect all three boxes  
        tree.query_in_direction(1.0, 0.4, 1.1, 0.6, 1.0, 0.0, 4.5, &mut results);
        
        // Swept area from (1,0.4)-(1.1,0.6) to (5.5,0.4)-(5.6,0.6) should intersect all boxes
        assert_eq!(results.len(), 3);
        assert!(results.contains(&0), "Should contain close box");
        assert!(results.contains(&1), "Should contain far box");
        assert!(results.contains(&2), "Should contain middle box");
    }

    #[test]
    fn test_containment_queries_comprehensive() {
        let mut tree = HilbertRTreeLeg::new();
        // add(min_x, min_y, max_x, max_y)
        tree.add(0.0, 0.0, 10.0, 10.0);  // Large container box
        tree.add(2.0, 2.0, 4.0, 4.0);    // Small box inside
        tree.add(8.0, 8.0, 12.0, 12.0);  // Overlapping box
        tree.add(-1.0, -1.0, 1.0, 1.0);  // Partially outside box
        tree.build();
        
        let mut results = Vec::new();
        
        // Test query_contain - which boxes contain the query rectangle (3,3) to (3.5,3.5)
        tree.query_contain(3.0, 3.0, 3.5, 3.5, &mut results);
        // The large box (0,0) to (10,10) should contain this rectangle
        assert!(results.contains(&0), "Large box should contain the query rectangle");
        assert!(results.len() >= 1, "At least one box should contain the query rectangle");
        
        results.clear();
        
        // Test query_contained_within - which boxes are contained by the query rectangle (1,1) to (9,9)
        tree.query_contained_within(1.0, 1.0, 9.0, 9.0, &mut results);
        assert_eq!(results.len(), 1);
        assert!(results.contains(&1), "Small box should be contained by the query rectangle");
        
        results.clear();
        
        // Test edge case - query rectangle exactly matches a box
        tree.query_contain(2.0, 2.0, 4.0, 4.0, &mut results);
        assert!(results.contains(&0), "Large container should contain exact match");
        assert!(results.contains(&1), "Box should contain itself (exact match)");
    }

    #[test]
    fn test_internal_methods_coverage() {
        let mut tree = HilbertRTreeLeg::new();
        tree.add(0.0, 1.0, 0.0, 1.0);
        tree.add(2.0, 3.0, 2.0, 3.0);
        tree.build();
        
        // Test query_within_distance_internal with sorting
        let candidates = tree.query_within_distance_internal(1.5, 1.5, 2.0, true);
        assert!(candidates.len() > 0, "Should find candidates within distance");
        
        // Verify sorting (first element should have smaller or equal distance than second)
        if candidates.len() > 1 {
            assert!(candidates[0].0 <= candidates[1].0, "Results should be sorted by distance");
        }
        
        // Test query_within_distance_internal without sorting
        let candidates_unsorted = tree.query_within_distance_internal(1.5, 1.5, 2.0, false);
        assert_eq!(candidates.len(), candidates_unsorted.len(), "Same number of candidates regardless of sorting");
        
        // Test query_nearest_internal
        let nearest_candidates = tree.query_nearest_internal(0.5, 0.5, Some(1));
        assert_eq!(nearest_candidates.len(), 1, "Should return exactly 1 nearest candidate");
        
        let all_nearest = tree.query_nearest_internal(0.5, 0.5, None);
        assert_eq!(all_nearest.len(), 2, "Should return all candidates when k=None");
        
        // Test query_in_direction_swept_internal with various parameters
        let dir_candidates = tree.query_in_direction_swept_internal(
            0.0, 0.5, 0.0, 0.5, 1.0, 0.0, 3.0, true
        );
        // Just verify we get some candidates (length check is sufficient)
        
        let dir_candidates_no_distances = tree.query_in_direction_swept_internal(
            0.0, 0.5, 0.0, 0.5, 1.0, 0.0, 3.0, false  
        );
        assert_eq!(dir_candidates.len(), dir_candidates_no_distances.len(), "Same number of candidates regardless of distance calculation");
    }

    #[test]
    fn test_performance_edge_cases() {
        let mut tree = HilbertRTreeLeg::new();
        
        // Add many overlapping boxes to test performance of intersection queries
        for i in 0..100 {
            let offset = i as f64 * 0.01;
            tree.add(offset, offset, offset + 1.0, offset + 1.0);
        }
        tree.build();
        
        let mut results = Vec::new();
        
        // Query that intersects with many boxes
        tree.query_intersecting(0.5, 0.5, 0.6, 0.6, &mut results);
        assert!(results.len() > 10, "Should find many overlapping boxes");
        
        results.clear();
        
        // Test k functionality with many results
        tree.query_intersecting_k(0.5, 0.5, 0.6, 0.6, 5, &mut results);
        assert_eq!(results.len(), 5, "Should respect k limit even with many candidates");
        
        // Test nearest k with many boxes
        results.clear();
        tree.query_nearest_k(0.5, 0.5, 10, &mut results);
        assert_eq!(results.len(), 10, "Should return exactly k nearest boxes");
    }

    #[test]
    fn test_boundary_precision_cases() {
        let mut tree = HilbertRTreeLeg::new();
        // add(min_x, min_y, max_x, max_y)
        // Create boxes with very close boundaries
        tree.add(0.0, 0.0, 1.0, 1.0);
        tree.add(1.0, 0.0, 2.0, 1.0);  // Shares edge with first box
        tree.add(0.0, 1.0, 1.0, 2.0);  // Shares edge with first box
        tree.build();
        
        let mut results = Vec::new();
        
        // Query exactly on the shared boundary - this point is contained by multiple boxes
        tree.query_intersecting(1.0, 1.0, 1.0, 1.0, &mut results);
        assert_eq!(results.len(), 3, "Point query on boundary should find all boxes that contain the point");
        
        results.clear();
        
        // Query spanning the boundary
        tree.query_intersecting(0.9, 0.9, 1.1, 1.1, &mut results);
        assert_eq!(results.len(), 3, "Query spanning boundaries should find all intersecting boxes");
        
        results.clear();
        
        // Test circle query at boundary
        tree.query_circle(1.0, 1.0, 0.1, &mut results);
        assert!(results.len() >= 1, "Circle at boundary should intersect at least one box");
    }
}