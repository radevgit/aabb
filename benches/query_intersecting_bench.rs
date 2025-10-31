//! Benchmark for hierarchical `query_intersecting` performance
//!
//! This benchmark measures the performance of the `query_intersecting` method
//! on a `HilbertRTree2` (hierarchical) with 1M randomly distributed bounding boxes.
//! Queries are performed with varying size categories (10%, 1%, 0.01%).

use aabb::HilbertRTree;
use rand::Rng;
use rand::SeedableRng;
use std::time::Instant;

/// Generate a random bounding box with the given size
/// Coordinate space: 100x100 (matching C++ benchmark)
/// Box size is variable UP TO the given `max_size`
fn add_random_box<R: Rng>(rng: &mut R, boxes: &mut Vec<f64>, max_size: f64) {
    let min_x = rng.random_range(0.0..(100.0 - max_size));
    let min_y = rng.random_range(0.0..(100.0 - max_size));
    let box_width = rng.random_range(0.0..max_size);
    let box_height = rng.random_range(0.0..max_size);
    let max_x = min_x + box_width;
    let max_y = min_y + box_height;
    
    boxes.push(min_x);
    boxes.push(min_y);
    boxes.push(max_x);
    boxes.push(max_y);
}

/// Benchmark search operations with different query box sizes
fn bench_search(
    tree: &HilbertRTree,
    boxes: &[f64],
    num_tests: usize,
    percentage_str: &str,
) {
    let mut results = Vec::new();
    let start = Instant::now();
    
    for chunk in boxes.chunks(4) {
        if chunk.len() == 4 {
            results.clear();
            tree.query_intersecting(chunk[0], chunk[1], chunk[2], chunk[3], &mut results);
        }
    }
    
    let elapsed = start.elapsed();
    println!(
        "{} searches {}%: {}ms",
        num_tests,
        percentage_str,
        elapsed.as_millis()
    );
}

/// Benchmark K-nearest neighbor queries
fn bench_neighbors(
    tree: &HilbertRTree,
    coords: &[f64],
    num_tests: usize,
    k: usize,
) {
    let mut results = Vec::new();
    let start = Instant::now();
    
    for i in 0..num_tests {
        results.clear();
        let x = coords[4 * i];
        let y = coords[4 * i + 1];
        tree.query_nearest_k(x, y, k, &mut results);
    }
    
    let elapsed = start.elapsed();
    println!(
        "{} searches of {} neighbors: {}ms",
        num_tests,
        k,
        elapsed.as_millis()
    );
}


fn main() {
    println!("AABB Hierarchical Hilbert R-tree Benchmark");
    println!("=========================================\n");
    
    let num_items = 1_000_000;
    let num_tests = 1_000;
    
    // Create MT19937 RNG with fixed seed for reproducibility
    let seed = 95756739_u64;
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    
    // Generate random boxes for indexing (coordinate space: 100x100)
    let mut coords = Vec::new();
    for _ in 0..num_items {
        add_random_box(&mut rng, &mut coords, 1.0);
    }
    
    // Generate test query boxes with different sizes (matching C++ benchmark)
    let mut boxes_100 = Vec::new();  // 10% coverage: sqrt(0.1) * 100
    let mut boxes_10 = Vec::new();   // 1% coverage: 10.0
    let mut boxes_1 = Vec::new();    // 0.01% coverage: 1.0
    let mut boxes_50 = Vec::new();   // 50% coverage: sqrt(0.5) * 100
    let mut boxes_100_full = Vec::new();  // 100% coverage: full space
    
    for _ in 0..num_tests {
        // Size to cover 100% of space: covers entire 100x100
        boxes_100_full.push(0.0);
        boxes_100_full.push(0.0);
        boxes_100_full.push(100.0);
        boxes_100_full.push(100.0);
        
        // Size to cover 50% of space: sqrt(0.5) * 100 ≈ 70.71
        add_random_box(&mut rng, &mut boxes_50, (0.5_f64).sqrt() * 100.0);
        // Size to cover 10% of space: sqrt(0.1) * 100 ≈ 31.62
        add_random_box(&mut rng, &mut boxes_100, (0.1_f64).sqrt() * 100.0);
        // Size to cover 1% of space: 10.0
        add_random_box(&mut rng, &mut boxes_10, 10.0);
        // Size to cover 0.01% of space: 1.0
        add_random_box(&mut rng, &mut boxes_1, 1.0);
    }
    
    // Build index
    println!("Building index with {} items...", num_items);
    let start = Instant::now();
    let mut tree = HilbertRTree::with_capacity(num_items);
    
    for chunk in coords.chunks(4) {
        if chunk.len() == 4 {
            tree.add(chunk[0], chunk[1], chunk[2], chunk[3]);
        }
    }
    
    tree.build();
    let build_time = start.elapsed();
    
    println!(
        "Index built in {:.2}ms\n",
        build_time.as_secs_f64() * 1000.0
    );
    
    // Run benchmarks with detailed timing
    println!("Running query benchmarks:");
    println!("-----------------------");
    bench_search(&tree, &boxes_100_full, num_tests, "100");
    bench_search(&tree, &boxes_50, num_tests, "50");
    bench_search(&tree, &boxes_100, num_tests, "10");
    bench_search(&tree, &boxes_10, num_tests, "1");
    bench_search(&tree, &boxes_1, num_tests, "0.01");
    println!();

    // Benchmark nearest neighbor queries
    println!("Running neighbor benchmarks:");
    println!("-----------------------");
    bench_neighbors(&tree, &coords, num_tests, 100);
    bench_neighbors(&tree, &coords, 1, num_items);
    bench_neighbors(&tree, &coords, num_items / 10, 1);
    println!();
}

/* 
cargo bench --bench query_intersecting_bench

Building index with 1000000 items...
Index built in 114.57ms

Running query benchmarks:
-----------------------
1000 searches 10%: 224ms
1000 searches 1%: 18ms
1000 searches 0.01%: 3ms

Running neighbor benchmarks:
-----------------------
1000 searches of 100 neighbors: 21.24ms
1 searches of 1000000 neighbors: 120.63ms
100000 searches of 1 neighbors: 883.86ms
________________________________________

After some optimizations:
Building index with 1000000 items...
Index built in 109.79ms

Running query benchmarks:
-----------------------
1000 searches 10%: 235ms
1000 searches 1%: 21ms
1000 searches 0.01%: 3ms

Running neighbor benchmarks:
-----------------------
1000 searches of 100 neighbors: 15ms
1 searches of 1000000 neighbors: 112ms
100000 searches of 1 neighbors: 567ms
_________________________________________
Opt 9
Index built in 89.83ms

Running query benchmarks:
-----------------------
1000 searches 100%: 2182ms
1000 searches 50%: 388ms
1000 searches 10%: 89ms
1000 searches 1%: 16ms
1000 searches 0.01%: 2ms

Running neighbor benchmarks:
-----------------------
1000 searches of 100 neighbors: 14ms
1 searches of 1000000 neighbors: 107ms
100000 searches of 1 neighbors: 564ms
_________________________________________
*/
