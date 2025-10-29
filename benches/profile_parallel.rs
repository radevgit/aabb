//! Parallel query benchmark to measure concurrent access performance
//!
//! This benchmark uses the SAME queries as profile_bench.rs so results are directly comparable.
//! It demonstrates that the HilbertRTree is safe to share across threads without interior
//! mutability, since queries only require &self (immutable borrow).

use aabb::HilbertRTree;
use rand::Rng;
use rand::SeedableRng;
use std::sync::Arc;
use std::thread;
use std::time::Instant;

fn main() {
    println!("AABB Parallel Query Benchmark (vs profile_bench.rs)");
    println!("===================================================\n");

    let num_items = 1_000_000;
    let num_tests = 1_000;
    let num_threads = 10;

    // Create MT19937 RNG with fixed seed for reproducibility (SAME as profile_bench.rs)
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

    // Build index (SAME as profile_bench.rs)
    println!("Building index...");
    let build_start = Instant::now();
    let mut tree = HilbertRTree::with_capacity(num_items);

    for chunk in coords.chunks(4) {
        if chunk.len() == 4 {
            tree.add(chunk[0], chunk[1], chunk[2], chunk[3]);
        }
    }
    tree.build();
    let build_time = build_start.elapsed();
    println!("  Index built in {:.2}ms\n", build_time.as_secs_f64() * 1000.0);

    // Wrap tree in Arc for safe sharing across threads
    let tree = Arc::new(tree);

    // Generate test queries (SAME as profile_bench.rs)
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

    // Parallel benchmark: Small queries
    println!("Profiling query_intersecting (parallel):");
    println!("{}", "-".repeat(40));

    let queries_small = Arc::new(test_queries_small);
    let parallel_start = Instant::now();

    let handles: Vec<_> = (0..num_threads)
        .map(|_| {
            let tree_clone = Arc::clone(&tree);
            let queries_clone = Arc::clone(&queries_small);

            thread::spawn(move || {
                let mut results = Vec::new();
                for (min_x, min_y, max_x, max_y) in queries_clone.iter() {
                    tree_clone.query_intersecting(*min_x, *min_y, *max_x, *max_y, &mut results);
                }
            })
        })
        .collect();

    for handle in handles {
        handle.join().unwrap();
    }

    let parallel_elapsed = parallel_start.elapsed();
    let total_queries = num_threads * num_tests;
    println!(
        "  {} small queries (parallel {}×{}):   {:.2}ms ({:.3}µs/query)",
        total_queries,
        num_threads,
        num_tests,
        parallel_elapsed.as_secs_f64() * 1000.0,
        parallel_elapsed.as_secs_f64() * 1_000_000.0 / total_queries as f64
    );

    // Parallel benchmark: Large queries
    let queries_large = Arc::new(test_queries_large);
    let parallel_start = Instant::now();

    let handles: Vec<_> = (0..num_threads)
        .map(|_| {
            let tree_clone = Arc::clone(&tree);
            let queries_clone = Arc::clone(&queries_large);

            thread::spawn(move || {
                let mut results = Vec::new();
                for (min_x, min_y, max_x, max_y) in queries_clone.iter() {
                    tree_clone.query_intersecting(*min_x, *min_y, *max_x, *max_y, &mut results);
                }
            })
        })
        .collect();

    for handle in handles {
        handle.join().unwrap();
    }

    let parallel_elapsed = parallel_start.elapsed();
    println!(
        "  {} large queries (parallel {}×{}):   {:.2}ms ({:.3}µs/query)",
        total_queries,
        num_threads,
        num_tests,
        parallel_elapsed.as_secs_f64() * 1000.0,
        parallel_elapsed.as_secs_f64() * 1_000_000.0 / total_queries as f64
    );

    // Parallel benchmark: query_nearest_k
    println!("\nProfiling query_nearest_k (parallel):");
    println!("{}", "-".repeat(40));

    let k_values = vec![1, 10, 100, 1000];
    let coords = Arc::new(coords);

    for k in k_values {
        let num_queries = if k == 1000 { 100 } else { num_tests };
        let total_parallel_queries = num_threads * num_queries;

        let parallel_start = Instant::now();

        let handles: Vec<_> = (0..num_threads)
            .map(|thread_id| {
                let tree_clone = Arc::clone(&tree);
                let coords_clone = Arc::clone(&coords);

                thread::spawn(move || {
                    let mut results = Vec::new();
                    for i in 0..num_queries {
                        let idx = (thread_id * num_queries + i) % (coords_clone.len() / 4);
                        let x = coords_clone[4 * idx];
                        let y = coords_clone[4 * idx + 1];
                        tree_clone.query_nearest_k(x, y, k, &mut results);
                    }
                })
            })
            .collect();

        for handle in handles {
            handle.join().unwrap();
        }

        let parallel_elapsed = parallel_start.elapsed();
        println!(
            "  {} queries k={} (parallel {}×{}):      {:.2}ms ({:.3}µs/query)",
            total_parallel_queries,
            k,
            num_threads,
            num_queries,
            parallel_elapsed.as_secs_f64() * 1000.0,
            parallel_elapsed.as_secs_f64() * 1_000_000.0 / total_parallel_queries as f64
        );
    }

    println!("\n{}", "=".repeat(40));
    println!("Conclusion:");
    println!("The HilbertRTree is safe to share across threads using Arc!");
    println!("All queries use &self → lock-free parallel access.");
}


/*
cargo bench --bench profile_parallel

Generating 1000000 random boxes...
  Generated in 26.95ms

Building index...
  Index built in 133.13ms

Profiling query_intersecting (parallel):
----------------------------------------
  10000 small queries (parallel 10×1000):   6.82ms (0.682µs/query)
  10000 large queries (parallel 10×1000):   4880.27ms (488.027µs/query)

Profiling query_nearest_k (parallel):
----------------------------------------
  10000 queries k=1 (parallel 10×1000):      11.07ms (1.107µs/query)
  10000 queries k=10 (parallel 10×1000):      11.86ms (1.186µs/query)
  10000 queries k=100 (parallel 10×1000):      24.00ms (2.400µs/query)
  1000 queries k=1000 (parallel 10×100):      15.13ms (15.128µs/query)


*/
