# AABB - Hilbert R-tree Spatial Index

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
aabb = "0.1.0"
```

### Basic Example

```rust
use aabb::prelude::*;

fn main() {
    let mut tree = HilbertRTree::new();
    
    // Add bounding boxes (min_x, max_x, min_y, max_y)
    tree.add(0.0, 1.0, 0.0, 1.0);
    tree.add(0.5, 1.5, 0.5, 1.5);
    tree.add(2.0, 3.0, 2.0, 3.0);
    
    // Build the spatial index
    tree.build();
    
    // Query for intersecting boxes
    let mut results = Vec::new();
    tree.query_intersecting(0.7, 1.3, 0.7, 1.3, &mut results);
    
    println!("Found {} intersecting boxes", results.len());
    // Results contains indices of boxes that intersect the query
}
```

## How it Works

The Hilbert R-tree stores bounding boxes in a flat array and sorts them by their Hilbert curve index (computed from box centers). This provides good spatial locality for most spatial queries while maintaining a simple, cache-friendly data structure.

## API Reference

- `HilbertRTree::new()` - Create a new empty tree
- `add(min_x, max_x, min_y, max_y)` - Add a bounding box
- `build()` - Build the spatial index (required before querying)
- `query_intersecting(min_x, max_x, min_y, max_y, results)` - Find intersecting boxes

## License

This project is licensed under the MIT License.