# AABB - Hilbert R-tree Spatial Index
[![Crates.io](https://img.shields.io/crates/v/aabb.svg?color=blue)](https://crates.io/crates/aabb)
[![Documentation](https://docs.rs/aabb/badge.svg)](https://docs.rs/aabb)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)


A Rust library providing a simple and efficient Hilbert R-tree implementation for spatial queries on axis-aligned bounding boxes (AABBs).

## Features

- **Hilbert Curve Ordering**: Uses Hilbert space-filling curve for improved spatial locality (inspired by [Flatbush](https://github.com/mourner/flatbush) algorithm)
- **AABB Intersection Queries**: Fast rectangular bounding box intersection testing
- **Zero-Copy**: Single contiguous buffer layout - safe for parallel queries with no allocations per query
- **Simple API**: Easy to use with minimal setup
- **Static Optimization**: Efficient for static or infrequently-modified spatial data

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
aabb = "0.6"
```

### Basic Example

```rust
use aabb::prelude::*;

fn main() {
    let mut tree = AABB::with_capacity(3);
    
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

### Point Cloud Example

```rust
use aabb::prelude::*;

fn main() {
    let mut tree = AABB::with_capacity(4);
    
    // Add points using the convenient add_point() method
    tree.add_point(0.0, 0.0);
    tree.add_point(1.0, 1.0);
    tree.add_point(2.0, 2.0);
    tree.add_point(5.0, 5.0);
    
    // Build the spatial index
    tree.build();
    
    // Query for points within a circular region (optimized for point data)
    let mut results = Vec::new();
    tree.query_circle_points(0.0, 0.0, 2.5, &mut results);
    
    println!("Found {} points within radius 2.5", results.len());
    // Results are sorted by distance (closest first)
    
    // Find K nearest points
    let mut results = Vec::new();
    tree.query_nearest_k_points(0.0, 0.0, 2, &mut results);
    println!("Found {} nearest points", results.len());
}
```

## How it Works

The Hilbert R-tree stores bounding boxes in a flat array and sorts them by their Hilbert curve index (computed from box centers). This provides good spatial locality for most spatial queries while maintaining a simple, cache-friendly data structure.

## API Reference

### Construction
- `HilbertRTree::new()` or `AABB::new()` - Create a new empty tree
- `HilbertRTree::with_capacity(capacity)` or `AABB::with_capacity(capacity)` - Create a new tree with preallocated capacity
- `HilbertRTreeI32::new()` or `AABBI32::new()` - Create a new empty tree
- `HilbertRTreeI32::with_capacity(capacity)` or `AABBI32::with_capacity(capacity)` - Create a new tree with preallocated capacity
- `add(min_x, min_y, max_x, max_y)` - `(f64, i32)` Add a bounding box
- `add_point(x, y)` - `(f64)` Add a point (convenience method - internally stores as (x, y, x, y))
- `build()` - `(f64, i32)` Build the spatial index (required before querying)
- `get(item_id)` - `(f64, i32)` Retrieve the bounding box for an item by its ID
- `get_point(item_id)` - `(f64)` Retrieve a point as (x, y) for items added with `add_point()`
- `save(path)` - `(f64, i32)` Save the built tree to a file for fast loading later
- `load(path)` - `(f64, i32)` Load a previously saved tree from a file

### Queries

#### Basic Spatial Queries
- `query_intersecting(min_x, min_y, max_x, max_y, results)` `(f64, i32)` - Find boxes that intersect a rectangle
- `query_intersecting_k(min_x, min_y, max_x, max_y, k, results)` `(f64, i32)` - Find first K intersecting boxes
- `query_point(x, y, results)` `(f64, i32)` - Find boxes that contain a point
- `query_contain(min_x, min_y, max_x, max_y, results)` `(f64, i32)` - Find boxes that contain a rectangle
- `query_contained_within(min_x, min_y, max_x, max_y, results)` `(f64, i32)` - Find boxes contained within a rectangle

#### Distance-Based Queries
- `query_nearest_k(x, y, k, results)` `(f64)` - Find K nearest boxes to a point
- `query_circle(center_x, center_y, radius, results)` `(f64)` - Find boxes intersecting a circular region

#### Point-Specific Optimized Queries
- `query_nearest_k_points(x, y, k, results)` `(f64)` - **Optimized** - Find K nearest points (stored as (x, x, y, y)) - ~30% faster than `query_nearest_k` for point clouds
- `query_circle_points(center_x, center_y, radius, results)` `(f64)` - **Optimized** - Find points within a circular region with distance-sorted results - ~30% faster than `query_circle` for point clouds

**Note:** Point-specific methods assume all items in the tree are stored as degenerate boxes (points) where `min_x == max_x` and `min_y == max_y`. For mixed data (both points and boxes), use the general methods instead. Results from point-specific queries are automatically sorted by distance (closest first).

#### Directional Queries
- `query_in_direction(rect_min_x, rect_min_y, rect_max_x, rect_max_y, direction_x, direction_y, distance, results)` `(f64)` - Find boxes intersecting a rectangle's movement path
- `query_in_direction_k(rect_min_x, rect_min_y, rect_max_x, rect_max_y, direction_x, direction_y, k, distance, results)` `(f64)` - Find K nearest boxes intersecting a rectangle's movement path

## Examples

Minimal examples for each query method are available in the `examples/` directory:

- `query_intersecting` - Find boxes intersecting a rectangle
- `query_intersecting_k` - Find K first intersecting boxes
- `query_point` - Find boxes containing a point
- `query_contain` - Find boxes containing a rectangle
- `query_contained_within` - Find boxes inside a rectangle
- `query_nearest_k` - Find K nearest boxes
- `query_circle` - Find boxes in a circular region
- `query_circle_points` - Find points in a circular region (optimized)
- `query_nearest_k_points` - Find K nearest points (optimized)
- `query_in_direction` - Find boxes in a movement path
- `query_in_direction_k` - Find K nearest in a movement path

Run any example with:
```bash
cargo run --example query_point
```

## Performance

- **Cache-Friendly**: Flat array storage with Hilbert curve ordering for good spatial locality
- **Static Optimization**: Optimized for static or infrequently-modified spatial data

```
Environment:
- OS: Ubuntu 24.04.3 LTS
- Processor: Intel Core i5-1240P
- Kernel: Linux 6.8.0-86-generic 
- CPU Frequency: ~1773-3500 MHz
> cargo bench --bench query_intersecting_bench
> cargo bench --bench query_intersecting_bench_i32

Building index with 1000000 items...
Index built in 89.28ms (f64)
Index built in 63.64ms (i32)

Running query benchmarks:
-----------------------
HilbertRTree::query_intersecting(f64)
1000 searches 100%: 2074ms
1000 searches 50%: 391ms
1000 searches 10%: 89ms
1000 searches 1%: 17ms
1000 searches 0.01%: 2ms

-----------------------
HilbertRTreeI32::query_intersecting(i32)
1000 searches 100%: 1397ms
1000 searches 50%: 197ms
1000 searches 10%: 42ms
1000 searches 1%: 6ms
1000 searches 0.01%: 0ms

Running neighbor benchmarks:
-----------------------
query_nearest_k(f64)
1000 searches of 100 neighbors: 13ms
1 searches of 1000000 neighbors: 107ms
100000 searches of 1 neighbors: 556ms

```


## Related Projects

NFP is part of the open-sourced [Nest2D](https://nest2d.com) projects collection.
