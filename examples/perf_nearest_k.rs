//! Performance profiling example for query_nearest_k
//! 
//! This example performs intensive query_nearest_k operations on a large spatial index.
//! Designed to be used with low-level profilers like `samply`:
//! 
//! ```bash
//! samply record cargo run --release --example perf_nearest_k
//! ```

use aabb::prelude::*;
use std::time::Instant;

fn main() {
    println!("Building large spatial index...");
    let mut tree = AABB::with_capacity(1_000_000);
    
    // Generate 1 million random bounding boxes
    let mut rng = 12345u64; // Simple LCG random number generator
    for _ in 0..1_000_000 {
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let x1 = ((rng >> 32) as f64 / u32::MAX as f64) * 1000.0;
        
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let y1 = ((rng >> 32) as f64 / u32::MAX as f64) * 1000.0;
        
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let size_x = ((rng >> 32) as f64 / u32::MAX as f64) * 50.0 + 1.0;
        
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let size_y = ((rng >> 32) as f64 / u32::MAX as f64) * 50.0 + 1.0;
        
        tree.add(x1, y1, x1 + size_x, y1 + size_y);
    }
    
    let build_start = Instant::now();
    tree.build();
    let build_duration = build_start.elapsed();
    
    let mut results = Vec::new();
    let query_start = Instant::now();
    let mut ccc = Vec::new();
    
    // Perform 10,000 K-nearest queries, finding 100 nearest neighbors each
    for _ in 0..10_000 {
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let point_x = ((rng >> 32) as f64 / u32::MAX as f64) * 1000.0;
        
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let point_y = ((rng >> 32) as f64 / u32::MAX as f64) * 1000.0;
        
        tree.query_nearest_k(point_x, point_y, 100, &mut results);
        ccc.push(results.len());
    }
    
    let query_duration = query_start.elapsed();
    
    println!(
        "\nCompleted 10,000 queries (k=100) in {:.2}ms ({:.2}µs per query)",
        query_duration.as_secs_f64() * 1000.0,
        query_duration.as_secs_f64() * 1_000_000.0 / 10_000.0
    );
    
    println!("Profile Summary:");
    println!("  Building:  {:.2}ms", build_duration.as_secs_f64() * 1000.0);
    println!(
        "  Querying:  {:.2}ms ({:.2}µs per query)",
        query_duration.as_secs_f64() * 1000.0,
        query_duration.as_secs_f64() * 1_000_000.0 / 10_000.0
    );
    println!("  Total:     {:.2}ms", (build_duration + query_duration).as_secs_f64() * 1000.0);
}

/*
cargo build --release --example perf_nearest_k
./target/release/examples/perf_nearest_k
samply record cargo run --release --example perf_nearest_k


Completed 10,000 queries (k=100)
Profile Summary:
  Building:  74.72ms
  Querying:  659.29ms (65.93µs per query)
  Total:     734.01ms
__________________________________________

*/
