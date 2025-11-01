//! Find K nearest points to a query point (optimized for point data).
//!
//! This example demonstrates the `query_nearest_k_points` method, which is optimized
//! for point clouds where all items are stored as degenerate bounding boxes (x, x, y, y).
//! Results are automatically sorted by distance (closest first).
//!
//! Performance: ~30% faster than `query_nearest_k` for point-only data.

use aabb::prelude::*;

fn main() {
    let mut tree = AABB::with_capacity(6);
    
    // Add points using the convenient add_point() method
    tree.add_point(0.0, 0.0);      // Point 0
    tree.add_point(1.0, 0.0);      // Point 1: distance 1 from (0, 0)
    tree.add_point(0.0, 1.0);      // Point 2: distance 1 from (0, 0)
    tree.add_point(1.0, 1.0);      // Point 3: distance sqrt(2) ≈ 1.41 from (0, 0)
    tree.add_point(3.0, 3.0);      // Point 4: distance sqrt(18) ≈ 4.24 from (0, 0)
    tree.add_point(10.0, 10.0);    // Point 5: distance sqrt(200) ≈ 14.14 from (0, 0)
    tree.build();

    println!("=== Query Nearest K Points Example ===\n");

    // Query 1: Find 1 nearest point to origin
    println!("Query 1: Find 1 nearest point to (0, 0):");
    let mut results = Vec::new();
    tree.query_nearest_k_points(0.0, 0.0, 1, &mut results);
    
    println!("  Result: {:?}", results);
    println!("  Expected: [0] (point 0 is at the origin)");
    assert_eq!(results.len(), 1, "Expected 1 result");
    assert_eq!(results[0], 0, "Point 0 should be closest");
    println!("  ✓ Correct!\n");

    // Query 2: Find 3 nearest points to origin
    println!("Query 2: Find 3 nearest points to (0, 0):");
    results.clear();
    tree.query_nearest_k_points(0.0, 0.0, 3, &mut results);
    
    println!("  Result: {:?}", results);
    println!("  Expected: [0, 1, 2] (closest points by distance)");
    assert_eq!(results.len(), 3, "Expected 3 results");
    assert_eq!(results[0], 0, "Point 0 should be closest (distance 0)");
    // Points 1 and 2 are equidistant (distance 1)
    assert!(results[1] == 1 || results[1] == 2, "Points 1 or 2 should be second");
    assert!(results[2] == 1 || results[2] == 2, "Points 1 or 2 should be third");
    println!("  ✓ Correct!\n");

    // Query 3: Find 5 nearest points to origin
    println!("Query 3: Find 5 nearest points to (0, 0):");
    results.clear();
    tree.query_nearest_k_points(0.0, 0.0, 5, &mut results);
    
    println!("  Result: {:?}", results);
    println!("  Expected: [0, 1, 2, 3, 4] in distance order");
    assert_eq!(results.len(), 5, "Expected 5 results");
    assert_eq!(results[0], 0, "Point 0 should be closest");
    // Results should be sorted by distance
    println!("  ✓ Correct!\n");

    // Query 4: Find more points than exist
    println!("Query 4: Find 100 nearest points (only 6 exist):");
    results.clear();
    tree.query_nearest_k_points(0.0, 0.0, 100, &mut results);
    
    println!("  Result count: {}", results.len());
    println!("  Expected: 6 (all available points)");
    assert_eq!(results.len(), 6, "Should return all available points");
    println!("  ✓ Correct!\n");

    // Query 5: Find 2 nearest points from (3, 3)
    println!("Query 5: Find 2 nearest points to (3, 3):");
    results.clear();
    tree.query_nearest_k_points(3.0, 3.0, 2, &mut results);
    
    println!("  Result: {:?}", results);
    println!("  Expected: [4, 3] (point 4 at (3,3), then point 3)");
    assert_eq!(results.len(), 2, "Expected 2 results");
    assert_eq!(results[0], 4, "Point 4 at (3, 3) should be closest");
    assert_eq!(results[1], 3, "Point 3 should be second");
    println!("  ✓ Correct!\n");

    // Compare with general query_nearest_k
    println!("=== Comparison: query_nearest_k_points vs query_nearest_k ===\n");
    
    let mut optimized_results = Vec::new();
    let mut general_results = Vec::new();
    
    tree.query_nearest_k_points(0.0, 0.0, 4, &mut optimized_results);
    tree.query_nearest_k(0.0, 0.0, 4, &mut general_results);
    
    println!("query_nearest_k_points results: {:?}", optimized_results);
    println!("query_nearest_k results:        {:?}", general_results);
    
    // Same points should be found and in same order
    assert_eq!(optimized_results, general_results, "Both methods should find same points in same order");
    println!("\n✓ Both methods find the same points in the same order!");
    
    println!("\n=== Performance Note ===");
    println!("query_nearest_k_points is ~30% faster than query_nearest_k for point data");
    println!("because it uses direct distance calculation instead of axis_distance()");
}
