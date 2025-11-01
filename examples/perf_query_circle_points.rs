//! Performance profiling example for query_circle_points
//! 
//! This example performs intensive query_circle_points operations on a large spatial index
//! with 1 million random points. Designed to be used with low-level profilers like `samply`:
//! 
//! ```bash
//! cargo run --release --example perf_query_circle_points
//! samply record cargo run --release --example perf_query_circle_points
//! ```

use aabb::prelude::*;
use std::time::Instant;

const NUM_POINTS: usize = 1_000_000;
const NUM_QUERIES: usize = 10_000;

fn main() {
    println!("Building spatial index with {} random points...", NUM_POINTS);
    let mut tree = AABB::with_capacity(NUM_POINTS);
    
    // Generate random points
    let mut rng = 12345u64; // Simple LCG random number generator
    for _ in 0..NUM_POINTS {
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let x = ((rng >> 32) as f64 / u32::MAX as f64) * 1000.0;
        
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let y = ((rng >> 32) as f64 / u32::MAX as f64) * 1000.0;
        
        tree.add_point(x, y);
    }
    
    let build_start = Instant::now();
    tree.build();
    let build_duration = build_start.elapsed();
    
    let mut results = Vec::new();
    let query_start = Instant::now();
    
    // Perform circular range queries for profiling
    for _ in 0..NUM_QUERIES {
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let center_x = ((rng >> 32) as f64 / u32::MAX as f64) * 1000.0;
        
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let center_y = ((rng >> 32) as f64 / u32::MAX as f64) * 1000.0;
        
        let radius = 5.0; // Query radius
        tree.query_circle_points(center_x, center_y, radius, &mut results);
    }
    
    let query_duration = query_start.elapsed();
    
    println!("Index built in {:.2}ms", build_duration.as_secs_f64() * 1000.0);
    println!("Completed {} queries in {:.2}ms ({:.2}µs per query)",
        NUM_QUERIES,
        query_duration.as_secs_f64() * 1000.0,
        query_duration.as_secs_f64() * 1_000_000.0 / NUM_QUERIES as f64
    );
}

/*
cargo run --release --example perf_query_circle_points

Building spatial index with 1000000 random points...
Index built in 71.81ms
Completed 10000 queries in 9447.02ms (944.70µs per query)

*/
