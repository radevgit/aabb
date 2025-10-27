//! Benchmark for query_intersecting performance
//!
//! This benchmark measures the performance of the `query_intersecting` method
//! on a HilbertRTree with 1M randomly distributed bounding boxes.
//! Queries are performed with varying size categories (100%, 10%, 1%).

use aabb::HilbertRTree;
use rand::Rng;
use rand::SeedableRng;
use std::time::Instant;

/// Generate a random bounding box with the given size
fn add_random_box<R: Rng>(rng: &mut R, boxes: &mut Vec<f64>, size: f64) {
    let min_x = rng.random_range(0.0..1000.0);
    let min_y = rng.random_range(0.0..1000.0);
    let max_x = min_x + size;
    let max_y = min_y + size;
    
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
    let mut total_results = 0usize;
    
    for chunk in boxes.chunks(4) {
        if chunk.len() == 4 {
            results.clear();
            tree.query_intersecting(chunk[0], chunk[1], chunk[2], chunk[3], &mut results);
            total_results += results.len();
        }
    }
    
    let elapsed = start.elapsed();
    println!(
        "{} searches {}: {:.2}ms (avg {:.0} results)",
        num_tests,
        percentage_str,
        elapsed.as_secs_f64() * 1000.0,
        total_results as f64 / num_tests as f64
    );
}

fn main() {
    println!("AABB Hilbert R-tree Benchmark");
    println!("============================\n");
    
    let num_items = 1_000_000;
    let num_tests = 1_000;
    
    // Create MT19937 RNG with fixed seed for reproducibility
    let seed = 95756739u64;
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    
    // Generate random boxes for indexing
    let mut coords = Vec::new();
    for _ in 0..num_items {
        add_random_box(&mut rng, &mut coords, 1.0);
    }
    
    // Generate test query boxes with different sizes
    let mut boxes_100 = Vec::new();
    let mut boxes_10 = Vec::new();
    let mut boxes_1 = Vec::new();
    let mut boxes_001 = Vec::new();
    
    for _ in 0..num_tests {
        // Size to cover 10% of space: sqrt(0.1) * 1000
        add_random_box(&mut rng, &mut boxes_100, (0.1_f64).sqrt() * 1000.0);
        // Size to cover 1% of space: sqrt(0.01) * 1000
        add_random_box(&mut rng, &mut boxes_10, (0.01_f64).sqrt() * 1000.0);
        // Size to cover 0.1% of space: sqrt(0.001) * 1000
        add_random_box(&mut rng, &mut boxes_1, (0.001_f64).sqrt() * 1000.0);
        // Size to cover 0.01% of space: sqrt(0.0001) * 1000
        add_random_box(&mut rng, &mut boxes_001, (0.0001_f64).sqrt() * 1000.0);
    }
    
    // Build index
    println!("Building index with {} items...", num_items);
    let start = Instant::now();
    let mut tree = HilbertRTree::new();
    
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
    bench_search(&tree, &boxes_100, num_tests, "10%");
    bench_search(&tree, &boxes_10, num_tests, "1%");
    bench_search(&tree, &boxes_1, num_tests, "0.1%");
    bench_search(&tree, &boxes_001, num_tests, "0.01%");
    println!();
    
    // Single query timing for reference
    println!("Single query timing:");
    println!("-----------------------");
    let mut results = Vec::new();
    let start = Instant::now();
    for _ in 0..1000 {
        results.clear();
        tree.query_intersecting(500.0, 500.0, 501.0, 501.0, &mut results);
    }
    let elapsed = start.elapsed();
    println!("1000 tiny queries: {:.2}ms ({:.3}ms per query)", 
        elapsed.as_secs_f64() * 1000.0,
        elapsed.as_secs_f64() * 1000.0 / 1000.0);
    println!();
}


/*

BASE
Building index with 1000000 items...
Index built in 74.86ms

Running query benchmarks:
-----------------------
1000 searches 10%: 2364.40ms (avg 70216 results)
1000 searches 1%: 1774.09ms (avg 9149 results)
1000 searches 0.1%: 1787.97ms (avg 1026 results)
1000 searches 0.01%: 1700.88ms (avg 120 results)

Single query timing:
-----------------------
1000 tiny queries: 6424.11ms (6.424ms per query)
________________________________________________

*/