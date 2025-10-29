//! Performance profiling example for query_intersecting
//! 
//! This example performs intensive query_intersecting operations on a large spatial index.
//! Designed to be used with low-level profilers like `samply`:
//! 
//! ```bash
//! samply record cargo run --release --example perf
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
    
    // println!("Index built in {:.2}ms", build_duration.as_secs_f64() * 1000.0);
    // println!("Index size: {} items", tree.len());
    // println!("\nPerforming intensive queries for profiling...");
    // println!("Each query window covers ~10% of the space");
    // println!("Total: 100,000 queries (adjust loop count for desired profile duration)\n");
    
    let mut results = Vec::new();
    let query_start = Instant::now();
    
    // Perform 100,000 intensive queries for profiling
    // Each query covers approximately 10% of the space
    for i in 0..100_000 {
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let center_x = ((rng >> 32) as f64 / u32::MAX as f64) * 1000.0;
        
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let center_y = ((rng >> 32) as f64 / u32::MAX as f64) * 1000.0;
        
        let query_size = 100.0; // 10% of space
        let min_x = (center_x - query_size / 2.0).max(0.0);
        let min_y = (center_y - query_size / 2.0).max(0.0);
        let max_x = (center_x + query_size / 2.0).min(1000.0);
        let max_y = (center_y + query_size / 2.0).min(1000.0);
        
        tree.query_intersecting(min_x, min_y, max_x, max_y, &mut results);
        
        // Print progress every 10,000 queries
        // if (i + 1) % 10_000 == 0 {
        //     println!(
        //         "  Completed {:>6} queries, last result count: {:>5}",
        //         i + 1,
        //         results.len()
        //     );
        // }
    }
    
    let query_duration = query_start.elapsed();
    
    println!(
        "\nCompleted 100,000 queries in {:.2}ms ({:.2}µs per query)",
        query_duration.as_secs_f64() * 1000.0,
        query_duration.as_secs_f64() * 1_000_000.0 / 100_000.0
    );
    
    println!("\nProfile Summary:");
    println!("  Building: {:.2}ms", build_duration.as_secs_f64() * 1000.0);
    println!("  Querying: {:.2}ms", query_duration.as_secs_f64() * 1000.0);
    println!("  Total:    {:.2}ms", (build_duration + query_duration).as_secs_f64() * 1000.0);
}


/*
Building large spatial index...
Index built in 97.08ms
Index size: 1000000 items

Performing intensive queries for profiling...
Each query window covers ~10% of the space
Total: 100,000 queries (adjust loop count for desired profile duration)

  Completed  10000 queries, last result count:  6929
  Completed  20000 queries, last result count: 15753
  Completed  30000 queries, last result count: 15787
  Completed  40000 queries, last result count: 15860
  Completed  50000 queries, last result count: 12047
  Completed  60000 queries, last result count: 15713
  Completed  70000 queries, last result count: 15362
  Completed  80000 queries, last result count: 15780
  Completed  90000 queries, last result count: 15832
  Completed 100000 queries, last result count: 15874

Completed 100,000 queries in 12426.00ms (124.26µs per query)

Profile Summary:
  Building: 97.08ms
  Querying: 12426.00ms
  Total:    12523.08ms
_______________________________________________________________________
Opt 1: - `get_box` `get_index` the temporary vect was created and destroyed. Now instead direct pointer
is used to return vox and index:
Completed 100,000 queries in 12394.71ms (123.95µs per query)

Profile Summary:
  Building: 98.37ms
  Querying: 12394.71ms
  Total:    12493.08ms
_______________________________________________________________________

Opt 4 - Vec for queue changed to VecDeque (39.5% speedup)
Completed 100,000 queries in 7504.98ms (75.05µs per query)

Profile Summary:
  Building: 101.63ms
  Querying: 7504.98ms
  Total:    7606.61ms
_______________________________________________________________________

*/