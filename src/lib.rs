//! # AABB - Hilbert R-tree Spatial Index
//!
//! A Rust library providing a simple and efficient Hilbert R-tree implementation 
//! for spatial queries on axis-aligned bounding boxes (AABBs).
//!
//! ## Features
//!
//! - **Hilbert Curve Ordering**: Uses Hilbert space-filling curve for improved spatial locality
//! - **AABB Intersection Queries**: Fast rectangular bounding box intersection testing
//! - **Simple API**: Easy to use with minimal setup
//! - **Static Optimization**: Efficient for static or infrequently-modified spatial data
//!
//! ## Quick Start
//!
//! ```rust
//! use aabb::prelude::*;
//!
//! // Create a new spatial index
//! let mut tree = HilbertRTree::new();
//!
//! // Add some bounding boxes (min_x, max_x, min_y, max_y)
//! tree.add(0.0, 2.0, 0.0, 2.0);    // Box 0: large box
//! tree.add(1.0, 3.0, 1.0, 3.0);    // Box 1: overlapping box
//! tree.add(5.0, 6.0, 5.0, 6.0);    // Box 2: distant box
//! tree.add(1.5, 2.5, 1.5, 2.5);    // Box 3: small box inside others
//!
//! // Build the spatial index (required before querying)
//! tree.build();
//!
//! // Query for boxes intersecting a region
//! let mut results = Vec::new();
//! tree.query_intersecting(1.2, 2.8, 1.2, 2.8, &mut results);
//!
//! // Results contains indices of intersecting boxes
//! println!("Found {} intersecting boxes: {:?}", results.len(), results);
//! // Output: Found 3 intersecting boxes: [0, 1, 3]
//!
//! // You can reuse the results vector for multiple queries
//! results.clear();
//! tree.query_intersecting(4.0, 7.0, 4.0, 7.0, &mut results);
//! println!("Found {} boxes in distant region: {:?}", results.len(), results);
//! // Output: Found 1 boxes in distant region: [2]
//! ```
//!
//! ## How It Works
//!
//! The Hilbert R-tree stores bounding boxes in a flat array and sorts them by their 
//! Hilbert curve index (computed from box centers). This provides good spatial locality 
//! for most spatial queries while maintaining a simple, cache-friendly data structure.
//!
//! The Hilbert curve is a space-filling curve that maps 2D coordinates to a 1D sequence
//! while preserving spatial locality - points that are close in 2D space tend to be 
//! close in the 1D Hilbert ordering.

pub mod hilbert_rtree;
pub mod prelude;

pub use hilbert_rtree::HilbertRTree;

pub fn add(left: u64, right: u64) -> u64 {
    left + right
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
