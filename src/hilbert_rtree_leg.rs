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
    /// Flat storage: (min_x, min_y, max_x, max_y) for each box
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
        HilbertRTreeLeg::with_capacity(0)
    }

    /// Creates a new Hilbert R-tree with preallocated capacity
    ///
    /// # Arguments
    /// * `capacity` - Expected number of bounding boxes to be added
    pub fn with_capacity(capacity: usize) -> Self {
        HilbertRTreeLeg {
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
    
    let xi = (safe_x.clamp(0.0, 1.0) * ((1u64 << max_level) as f64)) as u64;
    let yi = (safe_y.clamp(0.0, 1.0) * ((1u64 << max_level) as f64)) as u64;
    xy_to_hilbert(xi, yi, max_level)
}

/// Convert 2D coordinates to Hilbert curve index
fn xy_to_hilbert(x: u64, y: u64, order: u32) -> u64 {
    if order == 0 {
        return 0;
    }

    let mut d = 0u64;
    let mut s = 1u64 << (order.saturating_sub(1).min(63));

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