//! Detailed profiling benchmark to measure time spent in different query phases

use aabb::HilbertRTree;
use rand::Rng;
use rand::SeedableRng;
use std::time::Instant;

fn main() {
    println!("AABB Profiling Benchmark - Detailed Timing Analysis");
    println!("====================================================\n");

    let num_items = 1_000_000;
    let num_tests = 1_000;

    // Create MT19937 RNG with fixed seed for reproducibility
    let seed = 95756739_u64;
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);

    // Generate random boxes for indexing (coordinate space: 100x100)
    let mut coords = Vec::new();
    println!("Generating {} random boxes...", num_items);
    let gen_start = Instant::now();
    for _ in 0..num_items {
        let min_x = rng.random_range(0.0..100.0);
        let min_y = rng.random_range(0.0..100.0);
        let max_x = (min_x + rng.random_range(0.0..1.0_f64)).min(100.0);
        let max_y = (min_y + rng.random_range(0.0..1.0_f64)).min(100.0);

        coords.push(min_x);
        coords.push(min_y);
        coords.push(max_x);
        coords.push(max_y);
    }
    let gen_time = gen_start.elapsed();
    println!("  Generated in {:.2}ms\n", gen_time.as_secs_f64() * 1000.0);

    // Build index
    println!("Building index...");
    let build_start = Instant::now();
    let mut tree = HilbertRTree::with_capacity(num_items);

    // Phase 1: Adding items
    let add_start = Instant::now();
    for chunk in coords.chunks(4) {
        if chunk.len() == 4 {
            tree.add(chunk[0], chunk[1], chunk[2], chunk[3]);
        }
    }
    let add_time = add_start.elapsed();

    // Phase 2: Building tree (includes Hilbert computation)
    let build_phase_start = Instant::now();
    tree.build();
    let build_phase_time = build_phase_start.elapsed();

    let build_total = build_start.elapsed();
    println!("  Add items:        {:.2}ms", add_time.as_secs_f64() * 1000.0);
    println!("  Build tree:       {:.2}ms", build_phase_time.as_secs_f64() * 1000.0);
    println!("  Total build:      {:.2}ms\n", build_total.as_secs_f64() * 1000.0);

    // Profile query_intersecting
    println!("Profiling query_intersecting:");
    println!("{}", "-".repeat(40));

    // Generate test query boxes
    let mut test_queries_small = Vec::new();
    let mut test_queries_large = Vec::new();

    for _ in 0..num_tests {
        // Small query (0.01% coverage)
        let min_x = rng.random_range(0.0..99.0);
        let min_y = rng.random_range(0.0..99.0);
        test_queries_small.push((min_x, min_y, min_x + 1.0, min_y + 1.0));

        // Large query (10% coverage)
        let min_x = rng.random_range(0.0..69.0);
        let min_y = rng.random_range(0.0..69.0);
        test_queries_large.push((min_x, min_y, min_x + 31.62, min_y + 31.62));
    }

    // Small queries
    let mut results = Vec::new();
    let query_start = Instant::now();
    for (min_x, min_y, max_x, max_y) in &test_queries_small {
        tree.query_intersecting(*min_x, *min_y, *max_x, *max_y, &mut results);
    }
    let small_query_time = query_start.elapsed();
    println!(
        "  {} small queries (1%):    {:.2}ms ({:.3}µs/query)",
        num_tests,
        small_query_time.as_secs_f64() * 1000.0,
        small_query_time.as_secs_f64() * 1_000_000.0 / num_tests as f64
    );

    // Large queries
    let query_start = Instant::now();
    for (min_x, min_y, max_x, max_y) in &test_queries_large {
        tree.query_intersecting(*min_x, *min_y, *max_x, *max_y, &mut results);
    }
    let large_query_time = query_start.elapsed();
    println!(
        "  {} large queries (10%):   {:.2}ms ({:.3}µs/query)",
        num_tests,
        large_query_time.as_secs_f64() * 1000.0,
        large_query_time.as_secs_f64() * 1_000_000.0 / num_tests as f64
    );

    // Profile query_nearest_k
    println!("\nProfiling query_nearest_k:");
    println!("{}", "-".repeat(40));

    // K-nearest queries with different K values
    let k_values = vec![1, 10, 100, 1000];

    for k in k_values {
        let num_queries = if k == 1000 { 100 } else { num_tests };

        let query_start = Instant::now();
        for i in 0..num_queries {
            let x = coords[4 * i];
            let y = coords[4 * i + 1];
            tree.query_nearest_k(x, y, k, &mut results);
        }
        let elapsed = query_start.elapsed();

        println!(
            "  {} queries k={}:          {:.2}ms ({:.3}µs/query, avg {:.1} results)",
            num_queries,
            k,
            elapsed.as_secs_f64() * 1000.0,
            elapsed.as_secs_f64() * 1_000_000.0 / num_queries as f64,
            results.len() as f64
        );
    }

    // Profile query_point
    println!("\nProfiling query_point:");
    println!("{}", "-".repeat(40));

    let num_queries = num_tests * 10; // More iterations for fast queries
    let query_start = Instant::now();
    for i in 0..num_queries {
        results.clear();
        let x = coords[4 * (i % num_tests)];
        let y = coords[4 * (i % num_tests) + 1];
        tree.query_point(x, y, &mut results);
    }
    let elapsed = query_start.elapsed();
    println!(
        "  {} queries:               {:.2}ms ({:.3}µs/query, avg {:.1} results)",
        num_queries,
        elapsed.as_secs_f64() * 1000.0,
        elapsed.as_secs_f64() * 1_000_000.0 / num_queries as f64,
        results.len() as f64
    );

    // Profile query_intersecting_k
    println!("\nProfiling query_intersecting_k:");
    println!("{}", "-".repeat(40));

    let num_queries = num_tests * 10; // More iterations for fast queries
    let query_start = Instant::now();
    for i in 0..num_queries {
        results.clear();
        let (min_x, min_y, max_x, max_y) = test_queries_small[i % num_tests];
        tree.query_intersecting_k(min_x, min_y, max_x, max_y, 100, &mut results);
    }
    let elapsed = query_start.elapsed();
    println!(
        "  {} queries k=100 (small): {:.2}ms ({:.3}µs/query, avg {:.1} results)",
        num_queries,
        elapsed.as_secs_f64() * 1000.0,
        elapsed.as_secs_f64() * 1_000_000.0 / num_queries as f64,
        results.len() as f64
    );

    // Profile query_contain (find big boxes that contain the query area)
    // Create specific test queries that look for containing boxes
    // Use point-like queries within the space to find boxes that would contain them
    println!("\nProfiling query_contain:");
    println!("{}", "-".repeat(40));

    let num_queries = num_tests;
    let query_start = Instant::now();
    for _ in 0..num_queries {
        results.clear();
        // Query a tiny rectangle at random points
        let x = rng.random_range(0.0..100.0);
        let y = rng.random_range(0.0..100.0);
        tree.query_contain(x, y, x + 0.01, y + 0.01, &mut results);
    }
    let elapsed = query_start.elapsed();
    println!(
        "  {} queries (point-like):  {:.2}ms ({:.3}µs/query, avg {:.1} results)",
        num_queries,
        elapsed.as_secs_f64() * 1000.0,
        elapsed.as_secs_f64() * 1_000_000.0 / num_queries as f64,
        results.len() as f64
    );

    // Profile query_contained_within (find small boxes within the query area)
    println!("\nProfiling query_contained_within:");
    println!("{}", "-".repeat(40));

    let num_queries = num_tests;
    let query_start = Instant::now();
    for (min_x, min_y, max_x, max_y) in &test_queries_small {
        results.clear();
        tree.query_contained_within(*min_x, *min_y, *max_x, *max_y, &mut results);
    }
    let elapsed = query_start.elapsed();
    println!(
        "  {} queries (small):       {:.2}ms ({:.3}µs/query, avg {:.1} results)",
        num_queries,
        elapsed.as_secs_f64() * 1000.0,
        elapsed.as_secs_f64() * 1_000_000.0 / num_queries as f64,
        results.len() as f64
    );

    // Profile query_circle
    println!("\nProfiling query_circle:");
    println!("{}", "-".repeat(40));

    let query_start = Instant::now();
    for i in 0..num_tests {
        results.clear();
        let x = coords[4 * i];
        let y = coords[4 * i + 1];
        tree.query_circle(x, y, 5.0, &mut results);
    }
    let elapsed = query_start.elapsed();
    println!(
        "  {} queries (radius=5):    {:.2}ms ({:.3}µs/query, avg {:.1} results)",
        num_tests,
        elapsed.as_secs_f64() * 1000.0,
        elapsed.as_secs_f64() * 1_000_000.0 / num_tests as f64,
        results.len() as f64
    );

    // Profile query_in_direction
    println!("\nProfiling query_in_direction:");
    println!("{}", "-".repeat(40));

    let query_start = Instant::now();
    for (min_x, min_y, max_x, max_y) in &test_queries_small {
        results.clear();
        tree.query_in_direction(*min_x, *min_y, *max_x, *max_y, 1.0, 0.0, 10.0, &mut results);
    }
    let elapsed = query_start.elapsed();
    println!(
        "  {} queries (small):       {:.2}ms ({:.3}µs/query, avg {:.1} results)",
        num_tests,
        elapsed.as_secs_f64() * 1000.0,
        elapsed.as_secs_f64() * 1_000_000.0 / num_tests as f64,
        results.len() as f64
    );

    // Profile query_in_direction_k
    println!("\nProfiling query_in_direction_k:");
    println!("{}", "-".repeat(40));

    let query_start = Instant::now();
    for (min_x, min_y, max_x, max_y) in &test_queries_small {
        results.clear();
        tree.query_in_direction_k(*min_x, *min_y, *max_x, *max_y, 1.0, 0.0, 50, 10.0, &mut results);
    }
    let elapsed = query_start.elapsed();
    println!(
        "  {} queries k=50 (small):  {:.2}ms ({:.3}µs/query, avg {:.1} results)",
        num_tests,
        elapsed.as_secs_f64() * 1000.0,
        elapsed.as_secs_f64() * 1_000_000.0 / num_tests as f64,
        results.len() as f64
    );

    println!("\n{}", "=".repeat(50));
    println!("\nSummary:");
    println!("--------");
    println!("Build dominates: {:.1}%", (build_total.as_secs_f64() / (build_total.as_secs_f64() + small_query_time.as_secs_f64() + large_query_time.as_secs_f64())) * 100.0);
    println!("Queries (total): {:.2}ms", (small_query_time.as_secs_f64() + large_query_time.as_secs_f64()) * 1000.0);
}


/*
cargo bench --bench profile_bench

Generating 1000000 random boxes...
  Generated in 22.23ms

Building index...
  Add items:        15.19ms
  Build tree:       104.50ms
  Total build:      119.71ms

Profiling query_intersecting:
----------------------------------------
  1000 small queries (1%):    4.24ms (4.236µs/query)
  1000 large queries (10%):   1805.40ms (1805.402µs/query)

Profiling query_nearest_k:
----------------------------------------
  1000 queries k=1:          6.55ms (6.545µs/query, avg 1.0 results)
  1000 queries k=10:          6.71ms (6.706µs/query, avg 10.0 results)
  1000 queries k=100:          14.77ms (14.768µs/query, avg 100.0 results)
  100 queries k=1000:          8.34ms (83.359µs/query, avg 1000.0 results)

Profiling query_point:
----------------------------------------
  10000 queries:               11.42ms (1.142µs/query, avg 24.0 results)

Profiling query_intersecting_k:
----------------------------------------
  10000 queries k=100 (small): 17.95ms (1.795µs/query, avg 100.0 results)

Profiling query_contain:
----------------------------------------
  1000 queries (point-like):  1.76ms (1.759µs/query, avg 21.0 results)

Profiling query_contained_within:
----------------------------------------
  1000 queries (small):       2.67ms (2.674µs/query, avg 28.0 results)

Profiling query_circle:
----------------------------------------
  1000 queries (radius=5):    64.67ms (64.674µs/query, avg 8830.0 results)

Profiling query_in_direction:
----------------------------------------
  1000 queries (small):       14.61ms (14.606µs/query, avg 1752.0 results)

Profiling query_in_direction_k:
----------------------------------------
  1000 queries k=50 (small):  20.01ms (20.009µs/query, avg 50.0 results)

==================================================



*/