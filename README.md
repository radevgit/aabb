# AABB - Hilbert R-tree Spatial Index
[![Crates.io](https://img.shields.io/crates/v/aabb.svg?color=blue)](https://crates.io/crates/aabb)
[![Documentation](https://docs.rs/aabb/badge.svg)](https://docs.rs/aabb)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)


A Rust library providing efficient Hilbert R-tree implementation for spatial queries on axis-aligned bounding boxes (AABBs).

## Features

- **Hilbert Curve Ordering**: Uses Hilbert space-filling curve for improved spatial locality (inspired by [Flatbush](https://github.com/mourner/flatbush) algorithm)
- **AABB Intersection Queries**: Fast rectangular bounding box intersection testing
- **Zero-Copy**: Single contiguous buffer layout - safe for parallel queries with no allocations per query
- **Simple API**: Easy to use with minimal setup
- **Static Optimization**: Efficient for static or infrequently-modified spatial data. Changed data is handled by rebuilding the index (see [Changing Data](#changing-data))

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
aabb = "0.8"
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
    
    // Find K nearest points
    let mut results = Vec::new();
    tree.query_nearest_k_points(0.0, 0.0, 2, &mut results);
    println!("Found {} nearest points", results.len());
}
```

## How it Works

The Hilbert R-tree stores bounding boxes in a flat array and sorts them by their Hilbert curve index (computed from box centers). This provides good spatial locality for most spatial queries while maintaining a simple, cache-friendly data structure.

### Hilbert Curve Spatial Ordering

The Hilbert space-filling curve is a continuous fractal curve that visits every cell in a 2D grid exactly once, maintaining proximity in space:

![Hilbert Curve](docs/hilbert_curve_grid.svg)

The curve preserves spatial locality - points close to each other in 2D space tend to be close along the Hilbert curve order. This property makes the flat array layout extremely cache-friendly for spatial queries.


## Changing Data

The index is built in one pass and there is no way to move or remove a single item. When your data changes, rebuild the index. A full rebuild takes about 5 ms for 100,000 items.

This works the same for `AABB` (f64) and `AABBI32`:

- **Add and rebuild**: call `add()` for the new items, then `build()` again. Every item keeps the id `add()` gave it. Queries return no results between `add()` and the next `build()`.
- **Clear and refill**: call `clear()`, add the current items and `build()`. This reuses the tree's memory, which suits data rebuilt every frame. Ids start from 0 again after `clear()`.

```rust
use aabb::prelude::*;

fn main() {
    let mut tree = AABB::with_capacity(100);

    for frame in 0..3 {
        // Rebuild the index from the current positions
        tree.clear();
        for i in 0..100 {
            let x = i as f64 + frame as f64; // objects move each frame
            tree.add(x, 0.0, x + 1.0, 1.0);  // id == i, every frame
        }
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(0.0, 0.0, 5.0, 1.0, &mut results);
        println!("frame {frame}: {} boxes near the origin", results.len());
    }
}
```

`build()` on a tree that has not changed since the last build does nothing. `get(id)` builds a small id lookup table the first time it is called after a build (4 bytes per item), so repeated calls are fast.

## API Reference

### Construction
- `HilbertRTree::new()` or `AABB::new()` - Create a new empty tree
- `HilbertRTree::with_capacity(capacity)` or `AABB::with_capacity(capacity)` - Create a new tree with preallocated capacity
- `HilbertRTreeI32::new()` or `AABBI32::new()` - Create a new empty tree
- `HilbertRTreeI32::with_capacity(capacity)` or `AABBI32::with_capacity(capacity)` - Create a new tree with preallocated capacity
- `add(min_x, min_y, max_x, max_y)` - `(f64, i32)` Add a bounding box
- `add_point(x, y)` - `(f64)` Add a point (convenience method - internally stores as (x, y, x, y))
- `build()` - `(f64, i32)` Build the spatial index (required before querying). It can be called again after more `add()` calls; items keep their ids
- `clear()` - `(f64, i32)` Remove all items but keep the allocated memory, so the tree can be refilled and rebuilt. Ids start from 0 again
- `get(item_id)` - `(f64, i32)` Retrieve the bounding box for an item by its ID
- `get_point(item_id)` - `(f64)` Retrieve a point as (x, y) for items added with `add_point()`
- `save(path)` - `(f64, i32)` Save the built tree to a file for fast loading later
- `load(path)` - `(f64, i32)` Load a previously saved tree from a file

### Queries

#### Basic Spatial Queries
- `query_intersecting(min_x, min_y, max_x, max_y, results)` `(f64, i32)` - Find boxes that intersect a rectangle
- `query_intersecting_id(item_id, results)` `(f64, i32)` - Find boxes that intersect with a specific item already in the tree (by ID), excluding the item itself
- `query_intersecting_k(min_x, min_y, max_x, max_y, k, results)` `(f64, i32)` - Find first K intersecting boxes
- `query_point(x, y, results)` `(f64, i32)` - Find boxes that contain a point
- `query_contain(min_x, min_y, max_x, max_y, results)` `(f64, i32)` - Find boxes that contain a rectangle
- `query_contained_within(min_x, min_y, max_x, max_y, results)` `(f64, i32)` - Find boxes contained within a rectangle

#### Distance-Based Queries
- `query_nearest_k(x, y, k, results)` `(f64)` - Find K nearest boxes to a point
- `query_circle(center_x, center_y, radius, results)` `(f64)` - Find boxes intersecting a circular region

#### Point-Specific Optimized Queries
- `query_nearest_k_points(x, y, k, results)` `(f64)` - Find K nearest points (stored as (x, x, y, y)), sorted by distance
- `query_circle_points(center_x, center_y, radius, results)` `(f64)` - Find points within a circular region (optimized for point data)

**Note:** Point-specific methods assume all items in the tree are stored as degenerate boxes (points) where `min_x == max_x` and `min_y == max_y`. For mixed data (both points and boxes), use the general methods instead.

#### Directional Queries
- `query_in_direction(rect_min_x, rect_min_y, rect_max_x, rect_max_y, direction_x, direction_y, distance, results)` `(f64)` - Find boxes intersecting a rectangle's movement path
- `query_in_direction_k(rect_min_x, rect_min_y, rect_max_x, rect_max_y, direction_x, direction_y, k, distance, results)` `(f64)` - Find K nearest boxes intersecting a rectangle's movement path

## Examples

Minimal examples for each query method are available in the `examples/` directory:

- `query_intersecting` - Find boxes intersecting a rectangle
- `query_intersecting_id` - Find boxes intersecting a specific item already in the tree (by ID)
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
- **Static Optimization**: Optimized for static or infrequently-modified spatial data. Rebuilding is cheap (about 5 ms for 100,000 items), see [Changing Data](#changing-data)

```
Environment:
- OS: Ubuntu 26.04.1 LTS
- Processor: Intel Core i5-1240P
- Kernel: Linux 7.0.0-43-generic
- Rust: 1.100.0
- Results: median of 3 runs, 1,000,000 items, fixed seed
> cargo bench --bench profile_bench
> cargo bench --bench profile_bench_i32

Box Tree Queries (f64)
================
build box tree 1000000 items:        73.31ms
query_intersecting (100% coverage) - 1000 queries:     2254.808µs/query
query_intersecting (50% coverage)  - 1000 queries:      355.786µs/query
query_intersecting (10% coverage)  - 1000 queries:       82.899µs/query
query_intersecting (1% coverage)   - 1000 queries:       14.450µs/query
query_intersecting (0.01% coverage)- 1000 queries:        2.017µs/query
query_nearest_k (1000 searches of 100 neighbors):       12.897µs/query
query_nearest_k (1 search of 1000000 neighbors):       108.665ms
query_nearest_k (100000 searches of 1 neighbor):         5.169µs/query
query_nearest_k k=1 - 1000 queries:        5.164µs/query
query_nearest_k k=10 - 1000 queries:        5.833µs/query
query_nearest_k k=100 - 1000 queries:       12.792µs/query
query_nearest_k k=1000 - 100 queries:       77.114µs/query
query_point - 10000 queries:            1.005µs/query
query_intersecting_k k=100 - 10000 queries:        1.623µs/query
query_contain - 1000 queries:          1.151µs/query
query_contained_within - 1000 queries:        2.307µs/query
query_circle - 1000 queries (radius=5):       48.348µs/query
query_in_direction - 1000 queries:        13.101µs/query
query_in_direction_k k=50 - 1000 queries:       20.873µs/query

Point Cloud Queries
===================
build point cloud 1000000 points:        69.91ms
query_circle_points - 1000 queries (radius=5):       29.411µs/query
query_nearest_k_points k=1 - 1000 queries:        2.479µs/query
query_nearest_k_points k=10 - 1000 queries:        3.352µs/query
query_nearest_k_points k=100 - 1000 queries:       10.318µs/query
query_nearest_k_points k=1000 - 100 queries:       72.921µs/query


Box Tree Queries (i32)
================
build box tree 1000000 items:        55.45ms
query_intersecting (100% coverage) - 1000 queries:     1464.778µs/query
query_intersecting (50% coverage)  - 1000 queries:      265.412µs/query
query_intersecting (10% coverage)  - 1000 queries:       57.433µs/query
query_intersecting (1% coverage)   - 1000 queries:        7.472µs/query
query_intersecting (0.01% coverage)- 1000 queries:        0.581µs/query
query_intersecting_k k=100 - 10000 queries:        0.552µs/query
query_point - 10000 queries:            0.493µs/query
query_contain - 1000 queries:          0.112µs/query
query_contained_within - 1000 queries:        1.635µs/query
get() first call, builds id table - 1 call:          4.680ms
query_intersecting_id - 1000 queries:        0.735µs/query

```


## Related Projects

