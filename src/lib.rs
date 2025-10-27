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
//!
//! ## Quick Start
//!
//! ```rust
//! use aabb::prelude::*;
//!
//! // Create a new spatial index
//! let mut tree = HilbertRTree::new();
//!
//! // Add some bounding boxes (min_x, min_y, max_x, max_y)
//! tree.add(0.0, 0.0, 2.0, 2.0);    // Box 0: large box
//! tree.add(1.0, 1.0, 3.0, 3.0);    // Box 1: overlapping box
//! tree.add(5.0, 5.0, 6.0, 6.0);    // Box 2: distant box
//! tree.add(1.5, 1.5, 2.5, 2.5);    // Box 3: small box inside others
//!
//! // Build the spatial index (required before querying)
//! tree.build();
//!
//! // Query for boxes intersecting a region
//! let mut results = Vec::new();
//! // minx, miny, maxx, maxy
//! tree.query_intersecting(1.2, 1.2, 2.8, 2.8, &mut results);
//!
//! // Results contains indices of intersecting boxes
//! println!("Found {} intersecting boxes: {:?}", results.len(), results);
//! // Output: Found 3 intersecting boxes: [0, 1, 3]
//!
//! // You can reuse the results vector for multiple queries
//! results.clear();
//! tree.query_intersecting(4.0, 4.0, 7.0, 7.0, &mut results);
//! println!("Found {} boxes in distant region: {:?}", results.len(), results);
//! // Output: Found 1 boxes in distant region: [2]
//!
//! // Point queries - find boxes containing a specific point
//! results.clear();
//! tree.query_point(1.8, 1.8, &mut results);
//! println!("Point (1.8, 1.8) is in boxes: {:?}", results);
//!
//! // Containment queries - find boxes that completely contain a rectangle
//! results.clear();
//! tree.query_containing(1.2, 1.2, 1.8, 1.8, &mut results);
//! println!("Boxes containing rectangle: {:?}", results);
//!
//! // K-limited queries - find first K intersecting boxes for performance
//! results.clear();
//! tree.query_intersecting_k(0.0, 0.0, 3.0, 3.0, 2, &mut results);
//! println!("First 2 intersecting boxes: {:?}", results);
//!
//! // Nearest neighbor queries
//! let nearest = tree.query_nearest(2.0, 2.0);
//! println!("Nearest box to (2.0, 2.0): {:?}", nearest);
//!
//! // Distance-based queries - find boxes within 1.5 units
//! results.clear();
//! tree.query_within_distance(1.0, 1.0, 1.5, &mut results);
//! println!("Boxes within distance 1.5: {:?}", results);
//!
//! // Circular region queries
//! results.clear();
//! tree.query_circle(1.5, 1.5, 2.0, &mut results);
//! println!("Boxes in circle: {:?}", results);
//!
//! // K-nearest neighbor queries
//! results.clear();
//! tree.query_nearest_k(2.0, 2.0, 2, &mut results);
//! println!("2 nearest boxes to (2.0, 2.0): {:?}", results);
//!
//! // Directional queries - find boxes in rectangle's movement path
//! results.clear();
//! // Start with rectangle (1.5, 1.5) x (2.5, 2.5) and move right by distance 3
//! tree.query_in_direction(1.5, 1.5, 2.5, 2.5, 1.0, 0.0, 3.0, &mut results);
//! println!("Boxes intersecting movement path: {:?}", results);
//!
//! // Directional K-nearest queries - find K nearest boxes in movement path
//! results.clear();
//! tree.query_in_direction_k(1.5, 1.5, 2.5, 2.5, 1.0, 0.0, 2, 3.0, &mut results);
//! println!("2 nearest boxes in movement path: {:?}", results);
//!
//! ```
//!
//! ## Available Query Methods
//!
//! ### Basic Spatial Queries
//! - [`query_intersecting`] - Find boxes that intersect a rectangle
//! - [`query_intersecting_k`] - Find first K intersecting boxes
//! - [`query_point`] - Find boxes that contain a point
//! - [`query_containing`] - Find boxes that contain a rectangle  
//! - [`query_contained_by`] - Find boxes contained within a rectangle
//!
//! ### Distance-Based Queries
//! - [`query_nearest`] - Find the single nearest box to a point
//! - [`query_nearest_k`] - Find K nearest boxes to a point
//! - [`query_within_distance`] - Find boxes within distance of a point
//! - [`query_circle`] - Find boxes intersecting a circular region
//!
//! ### Directional Queries  
//! - [`query_in_direction`] - Find boxes intersecting a rectangle's movement path
//! - [`query_in_direction_k`] - Find K nearest boxes intersecting a rectangle's movement path
//!
//! [`query_intersecting`]: HilbertRTree::query_intersecting
//! [`query_intersecting_k`]: HilbertRTree::query_intersecting_k
//! [`query_point`]: HilbertRTree::query_point
//! [`query_containing`]: HilbertRTree::query_containing
//! [`query_contained_by`]: HilbertRTree::query_contained_by
//! [`query_nearest`]: HilbertRTree::query_nearest
//! [`query_nearest_k`]: HilbertRTree::query_nearest_k
//! [`query_within_distance`]: HilbertRTree::query_within_distance
//! [`query_circle`]: HilbertRTree::query_circle
//! [`query_in_direction`]: HilbertRTree::query_in_direction
//! [`query_in_direction_k`]: HilbertRTree::query_in_direction_k
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

/// Core Hilbert R-tree spatial index data structure
pub mod hilbert_rtree;
/// Spatial query implementations
pub mod queries;
/// Integration tests for the library
pub mod integration_test;
/// Prelude for convenient imports
pub mod prelude;

pub use hilbert_rtree::HilbertRTree;

/// Add two unsigned 64-bit integers
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
