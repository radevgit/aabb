# AABB - Hilbert R-tree Spatial Index
[![Crates.io](https://img.shields.io/crates/v/aabb.svg?color=blue)](https://crates.io/crates/aabb)
[![Documentation](https://docs.rs/aabb/badge.svg)](https://docs.rs/aabb)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)


A Rust library providing a simple and efficient Hilbert R-tree implementation for spatial queries on axis-aligned bounding boxes (AABBs).

## Features

- **Hilbert Curve Ordering**: Uses Hilbert space-filling curve for improved spatial locality
- **AABB Intersection Queries**: Fast rectangular bounding box intersection testing
- **Simple API**: Easy to use with minimal setup
- **Static Optimization**: Efficient for static or infrequently-modified spatial data

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
aabb = "0.2"
```

### Basic Example

```rust
use aabb::prelude::*;

fn main() {
    let mut tree = HilbertRTree::with_capacity(3);
    
    // Add bounding boxes (min_x, min_y, max_x, max_y)
    tree.add(0.0, 0.0, 1.0, 1.0);
    tree.add(0.5, 0.5, 1.5, 1.5);
    tree.add(2.0, 2.0, 3.0, 3.0);
    
    // Build the spatial index
    tree.build();
    
    // Query for intersecting boxes
    let mut results = Vec::new();
    // bbox: xmin, ymin, xmax, ymax 
    tree.query_intersecting(0.7, 0.7, 1.3, 1.3, &mut results);
    
    println!("Found {} intersecting boxes", results.len());
    // Results contains indices of boxes that intersect the query
}
```

## How it Works

The Hilbert R-tree stores bounding boxes in a flat array and sorts them by their Hilbert curve index (computed from box centers). This provides good spatial locality for most spatial queries while maintaining a simple, cache-friendly data structure.

## API Reference

### Construction
- `HilbertRTree::new()` - Create a new empty tree
- `HilbertRTree::with_capacity(capacity)` - Create a new tree with preallocated capacity
- `add(min_x, min_y, max_x, max_y)` - Add a bounding box
- `build()` - Build the spatial index (required before querying)

### Queries

#### Basic Spatial Queries
- `query_intersecting(min_x, min_y, max_x, max_y, results)` - Find boxes that intersect a rectangle
- `query_intersecting_k(min_x, min_y, max_x, max_y, k, results)` - Find first K intersecting boxes
- `query_point(x, y, results)` - Find boxes that contain a point
- `query_containing(min_x, min_y, max_x, max_y, results)` - Find boxes that contain a rectangle
- `query_contained_by(min_x, min_y, max_x, max_y, results)` - Find boxes contained within a rectangle

#### Distance-Based Queries
- `query_nearest_k(x, y, k, results)` - Find K nearest boxes to a point
- `query_nearest(x, y) -> Option<usize>` - Find the single nearest box to a point
- `query_within_distance(x, y, max_distance, results)` - Find boxes within distance of a point
- `query_circle(center_x, center_y, radius, results)` - Find boxes intersecting a circular region

#### Directional Queries
- `query_in_direction(rect_min_x, rect_min_y, rect_max_x, rect_max_y, direction_x, direction_y, distance, results)` - Find boxes intersecting a rectangle's movement path
- `query_in_direction_k(rect_min_x, rect_min_y, rect_max_x, rect_max_y, direction_x, direction_y, k, distance, results)` - Find K nearest boxes intersecting a rectangle's movement path

## Performance

- **Cache-Friendly**: Flat array storage with Hilbert curve ordering for good spatial locality
- **Static Optimization**: Optimized for static or infrequently-modified spatial data

## License

This project is licensed under the MIT License.