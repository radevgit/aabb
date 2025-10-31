//! Performance profiling example for tree building
//! 
//! This example focuses exclusively on benchmarking the build phase.
//! Designed to be used with low-level profilers like `samply`:
//! 
//! ```bash
//! samply record cargo run --release --example perf_build
//! ```

use aabb::prelude::*;
use std::time::Instant;

fn main() {
    println!("AABB Build Performance Benchmark");
    println!("=================================\n");
    
    println!("Generating 1,000,000 random bounding boxes...");
    let mut boxes = Vec::new();
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
        
        boxes.push((x1, y1, x1 + size_x, y1 + size_y));
    }
    
    println!("Running build phase benchmark 100 times for profiling...\n");
    
    let num_runs = 100;
    let mut total_add_time = std::time::Duration::ZERO;
    let mut total_build_time = std::time::Duration::ZERO;
    
    let overall_start = Instant::now();
    
    for run in 1..=num_runs {
        let build_start = Instant::now();
        let mut tree = AABB::with_capacity(1_000_000);
        
        // Add all boxes
        for (x1, y1, x2, y2) in &boxes {
            tree.add(*x1, *y1, *x2, *y2);
        }
        
        let add_duration = build_start.elapsed();
        total_add_time += add_duration;
        
        // Build the tree
        let build_phase_start = Instant::now();
        tree.build();
        let build_phase_duration = build_phase_start.elapsed();
        total_build_time += build_phase_duration;
        
        // if run % 10 == 0 {
        //     println!("Run {:3}/{}: Add {:.2}ms, Build {:.2}ms", 
        //         run, num_runs,
        //         add_duration.as_secs_f64() * 1000.0,
        //         build_phase_duration.as_secs_f64() * 1000.0
        //     );
        // }
    }
    
    let overall_duration = overall_start.elapsed();
    
    println!("\nBuild Summary ({} runs):", num_runs);
    println!("  Total Add time:   {:.2}ms", total_add_time.as_secs_f64() * 1000.0);
    println!("  Average Add:      {:.2}ms", total_add_time.as_secs_f64() * 1000.0 / num_runs as f64);
    println!("  Total Build time: {:.2}ms", total_build_time.as_secs_f64() * 1000.0);
    println!("  Average Build:    {:.2}ms", total_build_time.as_secs_f64() * 1000.0 / num_runs as f64);
    println!("  Overall time:     {:.2}ms", overall_duration.as_secs_f64() * 1000.0);
}


/*
cargo build --release --example perf_build
./target/release/examples/perf_build
samply record cargo run --release --example perf_build

Build Summary (100 runs):
  Total Add time:   1296.90ms
  Average Add:      12.97ms
  Total Build time: 9583.86ms
  Average Build:    95.84ms
  Overall time:     10988.86ms
_________________________________________________
OPT 6 - avoiding resize of initial data array
  Total Add time:   1091.36ms
  Average Add:      10.91ms
  Total Build time: 9616.03ms
  Average Build:    96.16ms
  Overall time:     10827.15ms
_________________________________________________
OPT 7 - replacing custom quicksort with sort_unstable_by_key
Note the Add phase also imrpves 50% somehow (memory layout?)
  Total Add time:   459.30ms
  Average Add:      4.59ms
  Total Build time: 7675.16ms
  Average Build:    76.75ms
  Overall time:     8231.33ms

__________________________________________________
OPT 8 - removed unecessary vector initializations (2.3%)
  Total Add time:   457.16ms
  Average Add:      4.57ms
  Total Build time: 7544.34ms
  Average Build:    75.44ms
  Overall time:     8124.56ms
___________________________________________________
OPT 9 - Calculate exact total nodes needed
  Total Add time:   397.41ms
  Average Add:      3.97ms
  Total Build time: 7442.48ms
  Average Build:    74.42ms
  Overall time:     7965.43ms

 */