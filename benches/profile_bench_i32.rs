//! Detailed profiling benchmark for i32 HilbertRTreeI32 coordinate version
//! Available queries: query_intersecting, query_intersecting_id, query_intersecting_k,
//!                   query_point, query_contain, query_contained_within
//! (Note: i32 version does not have query_nearest_k, query_circle, or point cloud variants)

use aabb::HilbertRTreeI32;
use rand::Rng;
use rand::SeedableRng;
use std::time::Instant;

/// Generate a random bounding box with variable size UP TO max_size
/// Coordinate space: 100x100 (scaled from f64)
fn add_random_box_i32<R: Rng>(rng: &mut R, boxes: &mut Vec<i32>, max_size: i32) {
    let min_x = rng.random_range(0..(100 - max_size));
    let min_y = rng.random_range(0..(100 - max_size));
    let box_width = rng.random_range(0..max_size);
    let box_height = rng.random_range(0..max_size);
    let max_x = min_x + box_width;
    let max_y = min_y + box_height;

    boxes.push(min_x);
    boxes.push(min_y);
    boxes.push(max_x);
    boxes.push(max_y);
}

fn main() {
    println!("AABB Profiling Benchmark (i32 version)");
    println!("======================================\n");

    let num_items = 1_000_000;
    let num_tests = 1_000;

    // Create MT19937 RNG with fixed seed for reproducibility
    let seed = 95756739_u64;
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);

    // Generate random boxes for indexing (coordinate space: 100x100)
    // Use add_random_box_i32() to match f64 version behavior
    let mut coords = Vec::new();
    for _ in 0..num_items {
        add_random_box_i32(&mut rng, &mut coords, 1);
    }

    // Build box tree index
    let mut tree = HilbertRTreeI32::with_capacity(num_items);
    for chunk in coords.chunks(4) {
        if chunk.len() == 4 {
            tree.add(chunk[0], chunk[1], chunk[2], chunk[3]);
        }
    }
    let build_start = Instant::now();
    tree.build();
    let build_total = build_start.elapsed();

    println!("Box Tree Queries");
    println!("================");
    println!(
        "build box tree {} items: {:>12.2}ms",
        num_items,
        build_total.as_secs_f64() * 1000.0
    );

    // Generate test query boxes with different coverage levels
    // Use same RNG for consistency with f64 version
    let mut boxes_100 = Vec::new(); // 100% coverage: full space
    let mut boxes_50 = Vec::new(); // 50% coverage
    let mut boxes_10 = Vec::new(); // 10% coverage
    let mut boxes_1 = Vec::new(); // 1% coverage
    let mut boxes_001 = Vec::new(); // 0.01% coverage

    for _ in 0..num_tests {
        // 100% coverage: full space
        boxes_100.push(0);
        boxes_100.push(0);
        boxes_100.push(100);
        boxes_100.push(100);

        // 50% coverage: sqrt(0.5) * 100 ≈ 71
        add_random_box_i32(&mut rng, &mut boxes_50, 71);

        // 10% coverage: sqrt(0.1) * 100 ≈ 32
        add_random_box_i32(&mut rng, &mut boxes_10, 32);

        // 1% coverage: 10
        add_random_box_i32(&mut rng, &mut boxes_1, 10);

        // 0.01% coverage: 1
        add_random_box_i32(&mut rng, &mut boxes_001, 1);
    }

    let mut results = Vec::with_capacity(100000);

    // Benchmark coverage-based queries
    // Coverage-based query_intersecting benchmarks (include query name in output)
    let query_start = Instant::now();
    for chunk in boxes_100.chunks(4) {
        if chunk.len() == 4 {
            tree.query_intersecting(chunk[0], chunk[1], chunk[2], chunk[3], &mut results);
        }
    }
    let elapsed = query_start.elapsed();
    println!(
        "query_intersecting (100% coverage) - {} queries: {:>12.3}µs/query",
        num_tests,
        elapsed.as_secs_f64() * 1_000_000.0 / num_tests as f64
    );

    let query_start = Instant::now();
    for chunk in boxes_50.chunks(4) {
        if chunk.len() == 4 {
            tree.query_intersecting(chunk[0], chunk[1], chunk[2], chunk[3], &mut results);
        }
    }
    let elapsed = query_start.elapsed();
    println!(
        "query_intersecting (50% coverage)  - {} queries: {:>12.3}µs/query",
        num_tests,
        elapsed.as_secs_f64() * 1_000_000.0 / num_tests as f64
    );

    let query_start = Instant::now();
    for chunk in boxes_10.chunks(4) {
        if chunk.len() == 4 {
            tree.query_intersecting(chunk[0], chunk[1], chunk[2], chunk[3], &mut results);
        }
    }
    let elapsed = query_start.elapsed();
    println!(
        "query_intersecting (50% coverage)  - {} queries: {:>12.3}µs/query",
        num_tests,
        elapsed.as_secs_f64() * 1_000_000.0 / num_tests as f64
    );

    let query_start = Instant::now();
    for chunk in boxes_1.chunks(4) {
        if chunk.len() == 4 {
            tree.query_intersecting(chunk[0], chunk[1], chunk[2], chunk[3], &mut results);
        }
    }
    let elapsed = query_start.elapsed();
    println!(
        "query_intersecting (1% coverage)   - {} queries: {:>12.3}µs/query",
        num_tests,
        elapsed.as_secs_f64() * 1_000_000.0 / num_tests as f64
    );

    let query_start = Instant::now();
    for chunk in boxes_001.chunks(4) {
        if chunk.len() == 4 {
            tree.query_intersecting(chunk[0], chunk[1], chunk[2], chunk[3], &mut results);
        }
    }
    let elapsed = query_start.elapsed();
    println!(
        "query_intersecting (0.01% coverage)- {} queries: {:>12.3}µs/query",
        num_tests,
        elapsed.as_secs_f64() * 1_000_000.0 / num_tests as f64
    );

    // Generate test query boxes for other query types (small, ~1% coverage)
    let mut test_queries_small = Vec::new();

    for _ in 0..num_tests {
        let min_x = rng.random_range(0..99);
        let min_y = rng.random_range(0..99);
        test_queries_small.push((min_x, min_y, min_x + 1, min_y + 1));
    }

    let mut results = Vec::new();

    // query_intersecting_k
    let num_queries = num_tests * 10;
    let query_start = Instant::now();
    for i in 0..num_queries {
        let (min_x, min_y, max_x, max_y) = test_queries_small[i % num_tests];
        tree.query_intersecting_k(min_x, min_y, max_x, max_y, 100, &mut results);
    }
    let elapsed = query_start.elapsed();
    println!(
        "query_intersecting_k k=100 - {} queries: {:>12.3}µs/query",
        num_queries,
        elapsed.as_secs_f64() * 1_000_000.0 / num_queries as f64
    );

    // query_point
    let num_queries = num_tests * 10;
    let query_start = Instant::now();
    for i in 0..num_queries {
        let x = coords[4 * (i % num_tests)];
        let y = coords[4 * (i % num_tests) + 1];
        tree.query_point(x, y, &mut results);
    }
    let elapsed = query_start.elapsed();
    println!(
        "query_point - {} queries:     {:>12.3}µs/query",
        num_queries,
        elapsed.as_secs_f64() * 1_000_000.0 / num_queries as f64
    );

    // query_contain
    let num_queries = num_tests;
    let query_start = Instant::now();
    for _ in 0..num_queries {
        let x = rng.random_range(0..100);
        let y = rng.random_range(0..100);
        tree.query_contain(x, y, x + 1, y + 1, &mut results);
    }
    let elapsed = query_start.elapsed();
    println!(
        "query_contain - {} queries:   {:>12.3}µs/query",
        num_queries,
        elapsed.as_secs_f64() * 1_000_000.0 / num_queries as f64
    );

    // query_contained_within
    let num_queries = num_tests;
    let query_start = Instant::now();
    for (min_x, min_y, max_x, max_y) in &test_queries_small {
        tree.query_contained_within(*min_x, *min_y, *max_x, *max_y, &mut results);
    }
    let elapsed = query_start.elapsed();
    println!(
        "query_contained_within - {} queries: {:>12.3}µs/query",
        num_queries,
        elapsed.as_secs_f64() * 1_000_000.0 / num_queries as f64
    );

    // query_intersecting_id (finds boxes intersecting a specific item by ID)
    let num_queries = num_tests;
    let query_start = Instant::now();
    for i in 0..num_queries {
        let item_id = i % num_items;
        if let Ok(_) = tree.query_intersecting_id(item_id, &mut results) {
            // Successfully queried
        }
    }
    let elapsed = query_start.elapsed();
    println!(
        "query_intersecting_id - {} queries: {:>12.3}µs/query",
        num_queries,
        elapsed.as_secs_f64() * 1_000_000.0 / num_queries as f64
    );
}

/*
cargo bench --bench profile_bench_i32

Box Tree Queries
================
build box tree 1000000 items:        55.23ms
query_intersecting (100% coverage) - 1000 queries:      1447.39ms
query_intersecting (50% coverage)  - 1000 queries:       255.19ms
query_intersecting (10% coverage)  - 1000 queries:        56.12ms
query_intersecting (1% coverage)   - 1000 queries:         8.18ms
query_intersecting (0.01% coverage)- 1000 queries:         0.74ms
query_intersecting_k k=100 - 10000 queries:        0.585µs/query
query_point - 10000 queries:            0.469µs/query
query_contain - 1000 queries:          0.301µs/query
query_contained_within - 1000 queries:        1.998µs/query
query_intersecting_id - 1000 queries:        0.351µs/query

*/

