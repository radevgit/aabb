//! Performance profiling example for tree building with configurable size
//! 
//! This example focuses exclusively on benchmarking the build phase.
//! The number of boxes is configurable via the constant at the top of the file.
//! 
//! Designed to be used with low-level profilers like `samply`:
//! 
//! ```bash
//! samply record cargo run --release --example perf_build_xl
//! ```

use aabb::prelude::*;
use std::time::Instant;

// Configurable constant: adjust this to change the number of boxes
const NUM_BOXES: usize = 10_000_000;
const NUM_RUNS: usize = 10;

fn format_number(n: usize) -> String {
    n.to_string()
        .chars()
        .rev()
        .collect::<Vec<_>>()
        .chunks(3)
        .map(|c| c.iter().collect::<String>())
        .collect::<Vec<_>>()
        .join("_")
        .chars()
        .rev()
        .collect()
}

fn main() {
    println!("AABB Build Performance Benchmark (XL)");
    println!("=====================================\n");
    
    println!("Generating {} random bounding boxes...", format_number(NUM_BOXES));
    let mut boxes = Vec::new();
    let mut rng = 12345u64; // Simple LCG random number generator
    
    for _ in 0..NUM_BOXES {
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
    
    println!("Running build phase benchmark {} times for profiling...\n", NUM_RUNS);
    
    let mut total_add_time = std::time::Duration::ZERO;
    let mut total_build_time = std::time::Duration::ZERO;
    
    let overall_start = Instant::now();
    
    for run in 1..=NUM_RUNS {
        let build_start = Instant::now();
        let mut tree = AABB::with_capacity(NUM_BOXES);
        
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
        //         run, NUM_RUNS,
        //         add_duration.as_secs_f64() * 1000.0,
        //         build_phase_duration.as_secs_f64() * 1000.0
        //     );
        // }
    }
    
    let overall_duration = overall_start.elapsed();
    
    println!("\nBuild Summary ({} boxes, {} runs):", format_number(NUM_BOXES), NUM_RUNS);
    println!("  Total Add time:   {:.2}ms", total_add_time.as_secs_f64() * 1000.0);
    println!("  Average Add:      {:.2}ms", total_add_time.as_secs_f64() * 1000.0 / NUM_RUNS as f64);
    println!("  Total Build time: {:.2}ms", total_build_time.as_secs_f64() * 1000.0);
    println!("  Average Build:    {:.2}ms", total_build_time.as_secs_f64() * 1000.0 / NUM_RUNS as f64);
    println!("  Overall time:     {:.2}ms", overall_duration.as_secs_f64() * 1000.0);
}

/*
cargo build --release --example perf_build_xl
./target/release/examples/perf_build_xl

Build Summary (10_000_000 boxes, 10 runs):
  Total Add time:   1165.65ms
  Average Add:      116.56ms
  Total Build time: 11639.82ms
  Average Build:    1163.98ms
  Overall time:     12917.58ms
__________________________________________
Build Summary (100_000_000 boxes, 1 runs):
  Total Add time:   1105.46ms
  Average Add:      1105.46ms
  Total Build time: 15697.15ms
  Average Build:    15697.15ms
  Overall time:     16880.30ms
__________________________________________
Build Summary (200_000_000 boxes, 1 runs):
  Total Add time:   2182.45ms
  Average Add:      2182.45ms
  Total Build time: 36055.33ms
  Average Build:    36055.33ms
  Overall time:     38390.39ms
_______________________________________________
OPT 9 
Build Summary (10_000_000 boxes, 10 runs):
  Total Add time:   1073.14ms
  Average Add:      107.31ms
  Total Build time: 11307.81ms
  Average Build:    1130.78ms
  Overall time:     12510.09ms

*/
