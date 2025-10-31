//! Performance profiling example for i32 variant query_intersecting
//! 
//! This example performs intensive query_intersecting operations on a large spatial index
//! using the i32 coordinate variant for comparison with the f64 version.
//! Designed to be used with low-level profilers like `samply`:
//! 
//! ```bash
//! samply record cargo run --release --example perf_i32
//! ```

use aabb::HilbertRTreeI32;
use std::time::Instant;

fn main() {
    println!("Building large spatial index (i32 variant)...");
    let mut tree = HilbertRTreeI32::with_capacity(1_000_000);
    
    // Generate 1 million random bounding boxes with i32 coordinates
    let mut rng = 12345u64; // Simple LCG random number generator
    for _ in 0..1_000_000 {
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let x1 = ((rng >> 32) as i32).abs() % 1000;
        
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let y1 = ((rng >> 32) as i32).abs() % 1000;
        
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let size_x = ((rng >> 32) as i32).abs() % 50 + 1;
        
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let size_y = ((rng >> 32) as i32).abs() % 50 + 1;
        
        tree.add(x1, y1, x1 + size_x, y1 + size_y);
    }
    
    let build_start = Instant::now();
    tree.build();
    let build_duration = build_start.elapsed();
    
    let mut results = Vec::new();
    let query_start = Instant::now();
    
    // Perform 100,000 intensive queries for profiling
    // Each query covers approximately 10% of the space
    for _ in 0..100_000 {
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let center_x = ((rng >> 32) as i32).abs() % 1000;
        
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let center_y = ((rng >> 32) as i32).abs() % 1000;
        
        let query_size = 100; // 10% of space
        let min_x = (center_x - query_size / 2).max(0);
        let min_y = (center_y - query_size / 2).max(0);
        let max_x = (center_x + query_size / 2).min(1000);
        let max_y = (center_y + query_size / 2).min(1000);
        
        tree.query_intersecting(min_x, min_y, max_x, max_y, &mut results);
    }
    
    let query_duration = query_start.elapsed();
    
    println!(
        "\nCompleted 100,000 queries in {:.2}ms ({:.2}µs per query)",
        query_duration.as_secs_f64() * 1000.0,
        query_duration.as_secs_f64() * 1_000_000.0 / 100_000.0
    );
    
    println!("\nProfile Summary (i32 variant):");
    println!("  Building: {:.2}ms", build_duration.as_secs_f64() * 1000.0);
    println!("  Querying: {:.2}ms", query_duration.as_secs_f64() * 1000.0);
    println!("  Total:    {:.2}ms", (build_duration + query_duration).as_secs_f64() * 1000.0);
}


/*
cargo build --release --example perf_i32
./target/release/examples/perf_i32

Base
Completed 100,000 queries in 11622.38ms (116.22µs per query)

Profile Summary (i32 variant):
  Building: 96.11ms
  Querying: 11622.38ms
  Total:    11718.49ms
______________________________________________________________
Opt 4 - Vec for queue changed to VecDeque (40% speedup)

Completed 100,000 queries in 6964.44ms (69.64µs per query)

Profile Summary (i32 variant):
  Building: 96.40ms
  Querying: 6964.44ms
  Total:    7060.84ms
______________________________________________________________
Opt 5 - use batches for get_box() 2                             

Completed 100,000 queries in 6389.59ms (63.90µs per query)

Profile Summary (i32 variant):
  Building: 97.27ms
  Querying: 6389.59ms
  Total:    6486.86ms
______________________________________________________________
Opt 7 - use rust sort methods instead of custom variant of quicksort

Profile Summary (i32 variant):
  Building: 70.44ms
  Querying: 6082.92ms
  Total:    6153.36ms
______________________________________________________________
Opt 9
Profile Summary (i32 variant):
  Building: 67.40ms
  Querying: 6092.90ms
  Total:    6160.30ms
______________________________________________________________

*/