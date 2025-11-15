//! Detailed profiling benchmark to measure time spent in different query phases

use aabb::HilbertRTree;
use rand::Rng;
use rand::SeedableRng;
use std::time::Instant;

/// Generate a random bounding box with variable size UP TO max_size
/// This matches the reference implementation in query_intersecting_bench.rs
fn add_random_box<R: Rng>(rng: &mut R, boxes: &mut Vec<f64>, max_size: f64) {
    let min_x = rng.random_range(0.0..(100.0 - max_size));
    let min_y = rng.random_range(0.0..(100.0 - max_size));
    let box_width = rng.random_range(0.0..max_size);
    let box_height = rng.random_range(0.0..max_size);
    let max_x = min_x + box_width;
    let max_y = min_y + box_height;
    
    boxes.push(min_x);
    boxes.push(min_y);
    boxes.push(max_x);
    boxes.push(max_y);
}

fn main() {
    println!("AABB Profiling Benchmark");
    println!("========================\n");

    let num_items = 1_000_000;
    let num_tests = 1_000;

    // Create MT19937 RNG with fixed seed for reproducibility
    let seed = 95756739_u64;
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);

    // Generate random boxes for indexing (coordinate space: 100x100)
    let mut coords = Vec::new();
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

    // Build box tree index
    let mut tree = HilbertRTree::with_capacity(num_items);
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
    println!("build box tree {} items: {:>12.2}ms", num_items, build_total.as_secs_f64() * 1000.0);

    // Generate test query boxes with different coverage levels
    // Use same RNG for consistency with reference benchmark
    let mut boxes_100 = Vec::new();      // 100% coverage: full space
    let mut boxes_50 = Vec::new();       // 50% coverage
    let mut boxes_10 = Vec::new();       // 10% coverage
    let mut boxes_1 = Vec::new();        // 1% coverage
    let mut boxes_001 = Vec::new();      // 0.01% coverage

    for _ in 0..num_tests {
        // 100% coverage: full space
        boxes_100.push(0.0);
        boxes_100.push(0.0);
        boxes_100.push(100.0);
        boxes_100.push(100.0);
        
        // 50% coverage: sqrt(0.5) * 100 ≈ 70.71
        add_random_box(&mut rng, &mut boxes_50, (0.5_f64).sqrt() * 100.0);
        
        // 10% coverage: sqrt(0.1) * 100 ≈ 31.62
        add_random_box(&mut rng, &mut boxes_10, (0.1_f64).sqrt() * 100.0);
        
        // 1% coverage: 10.0
        add_random_box(&mut rng, &mut boxes_1, 10.0);
        
        // 0.01% coverage: 1.0
        add_random_box(&mut rng, &mut boxes_001, 1.0);
    }

    let mut results = Vec::new();
    
    // Benchmark coverage-based queries
  // Coverage-based query_intersecting benchmarks (include query name in output)
  let query_start = Instant::now();
  for chunk in boxes_100.chunks(4) {
    if chunk.len() == 4 {
      results.clear();
      tree.query_intersecting(chunk[0], chunk[1], chunk[2], chunk[3], &mut results);
    }
  }
  let elapsed = query_start.elapsed();
  println!("query_intersecting (100% coverage) - {} queries: {:>12.2}ms", num_tests, elapsed.as_secs_f64() * 1000.0);

  let query_start = Instant::now();
  for chunk in boxes_50.chunks(4) {
    if chunk.len() == 4 {
      results.clear();
      tree.query_intersecting(chunk[0], chunk[1], chunk[2], chunk[3], &mut results);
    }
  }
  let elapsed = query_start.elapsed();
  println!("query_intersecting (50% coverage)  - {} queries: {:>12.2}ms", num_tests, elapsed.as_secs_f64() * 1000.0);

  let query_start = Instant::now();
  for chunk in boxes_10.chunks(4) {
    if chunk.len() == 4 {
      results.clear();
      tree.query_intersecting(chunk[0], chunk[1], chunk[2], chunk[3], &mut results);
    }
  }
  let elapsed = query_start.elapsed();
  println!("query_intersecting (10% coverage)  - {} queries: {:>12.2}ms", num_tests, elapsed.as_secs_f64() * 1000.0);

  let query_start = Instant::now();
  for chunk in boxes_1.chunks(4) {
    if chunk.len() == 4 {
      results.clear();
      tree.query_intersecting(chunk[0], chunk[1], chunk[2], chunk[3], &mut results);
    }
  }
  let elapsed = query_start.elapsed();
  println!("query_intersecting (1% coverage)   - {} queries: {:>12.2}ms", num_tests, elapsed.as_secs_f64() * 1000.0);

  let query_start = Instant::now();
  for chunk in boxes_001.chunks(4) {
    if chunk.len() == 4 {
      results.clear();
      tree.query_intersecting(chunk[0], chunk[1], chunk[2], chunk[3], &mut results);
    }
  }
  let elapsed = query_start.elapsed();
  println!("query_intersecting (0.01% coverage)- {} queries: {:>12.2}ms", num_tests, elapsed.as_secs_f64() * 1000.0);

    // Neighbor benchmarks (k-nearest queries with different counts)
    let mut neighbors_results = Vec::new();
    
    // 1000 searches of 100 neighbors
    let query_start = Instant::now();
    for i in 0..num_tests {
        neighbors_results.clear();
        let x = coords[4 * i];
        let y = coords[4 * i + 1];
        tree.query_nearest_k(x, y, 100, &mut neighbors_results);
    }
    let elapsed = query_start.elapsed();
    println!(                "1000 searches of 100 neighbors:                    {:>8}ms", elapsed.as_millis());
    
    // 1 search of 1000000 neighbors (all items)
    let query_start = Instant::now();
    neighbors_results.clear();
    let x = coords[0];
    let y = coords[1];
    tree.query_nearest_k(x, y, num_items, &mut neighbors_results);
    let elapsed = query_start.elapsed();
    println!("1 search of 1000000 neighbors:                     {:>8}ms", elapsed.as_millis());
    
    // 100000 searches of 1 neighbor
    let num_searches = num_items / 10;
    let query_start = Instant::now();
    for i in 0..num_searches {
        neighbors_results.clear();
        let x = coords[4 * (i % num_tests)];
        let y = coords[4 * (i % num_tests) + 1];
        tree.query_nearest_k(x, y, 1, &mut neighbors_results);
    }
    let elapsed = query_start.elapsed();
    println!("100000 searches of 1 neighbor:                     {:>8}ms", elapsed.as_millis());

    // Generate test query boxes for other query types
    let mut test_queries_small = Vec::new();

    for _ in 0..num_tests {
        // Small query (0.01% coverage)
        let min_x = rng.random_range(0.0..99.0);
        let min_y = rng.random_range(0.0..99.0);
        test_queries_small.push((min_x, min_y, min_x + 1.0, min_y + 1.0));
    }

    let mut results = Vec::new();

    // query_nearest_k with different K values
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
    println!("query_nearest_k k={} - {} queries: {:>12.3}µs/query", k, num_queries, elapsed.as_secs_f64() * 1_000_000.0 / num_queries as f64);
  }


    // query_point
    let num_queries = num_tests * 10;
    let query_start = Instant::now();
    for i in 0..num_queries {
        results.clear();
        let x = coords[4 * (i % num_tests)];
        let y = coords[4 * (i % num_tests) + 1];
        tree.query_point(x, y, &mut results);
    }
  let elapsed = query_start.elapsed();
  println!("query_point - {} queries:      {:>12.3}µs/query", num_queries, elapsed.as_secs_f64() * 1_000_000.0 / num_queries as f64);


    // query_intersecting_k
    let num_queries = num_tests * 10;
    let query_start = Instant::now();
    for i in 0..num_queries {
        results.clear();
        let (min_x, min_y, max_x, max_y) = test_queries_small[i % num_tests];
        tree.query_intersecting_k(min_x, min_y, max_x, max_y, 100, &mut results);
    }
  let elapsed = query_start.elapsed();
  println!("query_intersecting_k k=100 - {} queries: {:>12.3}µs/query", num_queries, elapsed.as_secs_f64() * 1_000_000.0 / num_queries as f64);


    // query_contain
    let num_queries = num_tests;
    let query_start = Instant::now();
    for _ in 0..num_queries {
        results.clear();
        let x = rng.random_range(0.0..100.0);
        let y = rng.random_range(0.0..100.0);
        tree.query_contain(x, y, x + 0.01, y + 0.01, &mut results);
    }
  let elapsed = query_start.elapsed();
  println!("query_contain - {} queries:    {:>12.3}µs/query", num_queries, elapsed.as_secs_f64() * 1_000_000.0 / num_queries as f64);


    // query_contained_within
    let num_queries = num_tests;
    let query_start = Instant::now();
    for (min_x, min_y, max_x, max_y) in &test_queries_small {
        results.clear();
        tree.query_contained_within(*min_x, *min_y, *max_x, *max_y, &mut results);
    }
  let elapsed = query_start.elapsed();
  println!("query_contained_within - {} queries: {:>12.3}µs/query", num_queries, elapsed.as_secs_f64() * 1_000_000.0 / num_queries as f64);


    // query_circle
    let query_start = Instant::now();
    for i in 0..num_tests {
        results.clear();
        let x = coords[4 * i];
        let y = coords[4 * i + 1];
        tree.query_circle(x, y, 5.0, &mut results);
    }
  let elapsed = query_start.elapsed();
  println!("query_circle - {} queries (radius=5): {:>12.3}µs/query", num_tests, elapsed.as_secs_f64() * 1_000_000.0 / num_tests as f64);


    // query_in_direction
    let query_start = Instant::now();
    for (min_x, min_y, max_x, max_y) in &test_queries_small {
        results.clear();
        tree.query_in_direction(*min_x, *min_y, *max_x, *max_y, 1.0, 0.0, 10.0, &mut results);
    }
  let elapsed = query_start.elapsed();
  println!("query_in_direction - {} queries:   {:>12.3}µs/query", num_tests, elapsed.as_secs_f64() * 1_000_000.0 / num_tests as f64);


    // query_in_direction_k
    let query_start = Instant::now();
    for (min_x, min_y, max_x, max_y) in &test_queries_small {
        results.clear();
        tree.query_in_direction_k(*min_x, *min_y, *max_x, *max_y, 1.0, 0.0, 50, 10.0, &mut results);
    }
  let elapsed = query_start.elapsed();
  println!("query_in_direction_k k=50 - {} queries: {:>12.3}µs/query", num_tests, elapsed.as_secs_f64() * 1_000_000.0 / num_tests as f64);

    println!("\n\nPoint Cloud Queries");
    println!("===================");

    // Build a tree with point data for point-specific queries
    let mut point_tree = HilbertRTree::with_capacity(num_items);
    for _ in 0..num_items {
        let x = rng.random_range(0.0..100.0);
        let y = rng.random_range(0.0..100.0);
        point_tree.add_point(x, y);
    }
    let build_point_start = Instant::now();
    point_tree.build();
    let build_point_time = build_point_start.elapsed();
    println!("build point cloud {} points: {:>12.2}ms", num_items, build_point_time.as_secs_f64() * 1000.0);


    // query_circle_points
    let query_start = Instant::now();
    for i in 0..num_tests {
        results.clear();
        let x = coords[4 * i];
        let y = coords[4 * i + 1];
        point_tree.query_circle_points(x, y, 5.0, &mut results);
    }
  let elapsed = query_start.elapsed();
  println!("query_circle_points - {} queries (radius=5): {:>12.3}µs/query", num_tests, elapsed.as_secs_f64() * 1_000_000.0 / num_tests as f64);


    // query_nearest_k_points with different K values
    let k_values = vec![1, 10, 100, 1000];
    for k in k_values {
        let num_queries = if k == 1000 { 100 } else { num_tests };
        let query_start = Instant::now();
        for i in 0..num_queries {
            let x = coords[4 * i];
            let y = coords[4 * i + 1];
            point_tree.query_nearest_k_points(x, y, k, &mut results);
        }
  let elapsed = query_start.elapsed();
  println!("query_nearest_k_points k={} - {} queries: {:>12.3}µs/query", k, num_queries, elapsed.as_secs_f64() * 1_000_000.0 / num_queries as f64);
    }
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
  1000 large queries (10%):   400.04ms (400.036µs/query)

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

Summary:
--------
Build dominates: 25.1%
Queries (total): 352.22ms
_________________________________________________________________________________
Opt 7 - use rust sorting instead of custom quicksort

Generating 1000000 random boxes...
  Generated in 30.14ms

Building index...
  Add items:        11.30ms
  Build tree:       84.25ms
  Total build:      95.57ms

Profiling query_intersecting:
----------------------------------------
  1000 small queries (1%):    4.35ms (4.349µs/query)
  1000 large queries (10%):   352.46ms (352.463µs/query)

Profiling query_nearest_k:
----------------------------------------
  1000 queries k=1:          5.81ms (5.811µs/query, avg 1.0 results)
  1000 queries k=10:          5.99ms (5.990µs/query, avg 10.0 results)
  1000 queries k=100:          12.94ms (12.944µs/query, avg 100.0 results)
  100 queries k=1000:          7.94ms (79.404µs/query, avg 1000.0 results)

Profiling query_point:
----------------------------------------
  10000 queries:               9.75ms (0.975µs/query, avg 24.0 results)

Profiling query_intersecting_k:
----------------------------------------
  10000 queries k=100 (small): 16.71ms (1.671µs/query, avg 100.0 results)

Profiling query_contain:
----------------------------------------
  1000 queries (point-like):  1.65ms (1.645µs/query, avg 21.0 results)

Profiling query_contained_within:
----------------------------------------
  1000 queries (small):       2.35ms (2.354µs/query, avg 28.0 results)

Profiling query_circle:
----------------------------------------
  1000 queries (radius=5):    46.70ms (46.697µs/query, avg 8830.0 results)

Profiling query_in_direction:
----------------------------------------
  1000 queries (small):       11.95ms (11.950µs/query, avg 1752.0 results)

Profiling query_in_direction_k:
----------------------------------------
  1000 queries k=50 (small):  19.75ms (19.746µs/query, avg 50.0 results)

==================================================

Summary:
--------
Build dominates: 21.1%
Queries (total): 356.81ms
___________________________________________________________________________________

Box Tree Queries
================
build box tree 1000000 items:        83.33ms
1000 queries small (1%):             4.090µs/query
1000 queries large (10%):          379.150µs/query
1000 queries k=1:                     6.236µs/query
1000 queries k=10:                     6.337µs/query
1000 queries k=100:                    14.114µs/query
100 queries k=1000:                    96.075µs/query
10000 queries:                         1.318µs/query
10000 queries k=100 (small):          2.291µs/query
1000 queries (point-like):           2.042µs/query
1000 queries (small):                3.695µs/query
1000 queries (radius=5):           57.058µs/query
1000 queries (small):               13.966µs/query
1000 queries k=50 (small):         23.477µs/query


Point Cloud Queries
===================
build point cloud 1000000 points:        72.51ms
1000 queries (radius=5):           26.980µs/query
1000 queries k=1:                     2.518µs/query
1000 queries k=10:                     3.481µs/query
1000 queries k=100:                    10.970µs/query
100 queries k=1000:                    73.031µs/query
_________________________________________________________________________
*/