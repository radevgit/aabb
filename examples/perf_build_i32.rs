//! Performance profiling example for i32 tree building
//! 
//! This example focuses exclusively on benchmarking the build phase for the i32 variant.
//! Designed to be used with low-level profilers like `samply`:
//! 
//! ```bash
//! samply record cargo run --release --example perf_build_i32
//! ```

use aabb::prelude::*;
use std::time::Instant;

fn main() {
    println!("AABB Build Performance Benchmark (i32 variant)");
    println!("==============================================\n");
    
    println!("Generating 1,000,000 random bounding boxes...");
    let mut boxes = Vec::new();
    let mut rng = 12345u64; // Simple LCG random number generator
    
    for _ in 0..1_000_000 {
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let x1 = ((rng >> 32) as i32) % 1000;
        
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let y1 = ((rng >> 32) as i32) % 1000;
        
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let size_x = ((rng >> 32) as i32) % 50 + 1;
        
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let size_y = ((rng >> 32) as i32) % 50 + 1;
        
        boxes.push((x1, y1, x1 + size_x, y1 + size_y));
    }
    
    println!("Running build phase benchmark 100 times for profiling...\n");
    
    let num_runs = 100;
    let mut total_add_time = std::time::Duration::ZERO;
    let mut total_build_time = std::time::Duration::ZERO;
    
    let overall_start = Instant::now();
    
    for _run in 1..=num_runs {
        let build_start = Instant::now();
        let mut tree = HilbertRTreeI32::with_capacity(1_000_000);
        
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
        
        // if _run % 10 == 0 {
        //     println!("Run {:3}/{}: Add {:.2}ms, Build {:.2}ms", 
        //         _run, num_runs,
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
cargo build --release --example perf_build_i32
./target/release/examples/perf_build_i32

Baseline (with introsort sort optimization):
  Total Add time:   413.49ms
  Average Add:      4.13ms
  Total Build time: 6507.64ms
  Average Build:    65.08ms
  Overall time:     6989.63ms
  ______________________________________________
OPT 8 - removed unecessary vector initializations (2.3%)
  Total Add time:   423.22ms
  Average Add:      4.23ms
  Total Build time: 6417.60ms
  Average Build:    64.18ms
  Overall time:     6895.65ms


*/
