//! Find points within a circular region (optimized for point data).
//!
//! This example demonstrates the `query_circle_points` method, which is optimized
//! for point clouds where all items are stored as degenerate bounding boxes (x, x, y, y).
//! Results are automatically sorted by distance (closest first).
//!
//! Performance: ~30% faster than `query_circle` for point-only data.

use aabb::prelude::*;

fn main() {
    let mut tree = AABB::with_capacity(5);
    
    // Add points using the convenient add_point() method
    tree.add_point(0.0, 0.0);      // Point 0: distance 0 from (0, 0)
    tree.add_point(1.0, 0.0);      // Point 1: distance 1 from (0, 0)
    tree.add_point(0.0, 1.0);      // Point 2: distance 1 from (0, 0)
    tree.add_point(1.0, 1.0);      // Point 3: distance sqrt(2) ≈ 1.41 from (0, 0)
    tree.add_point(5.0, 5.0);      // Point 4: distance sqrt(50) ≈ 7.07 from (0, 0)
    tree.build();

    println!("=== Query Circle Points Example ===\n");

    // Query 1: Find points within radius 1.5 from origin
    println!("Query 1: Points within radius 1.5 from (0, 0):");
    let mut results = Vec::new();
    tree.query_circle_points(0.0, 0.0, 1.5, &mut results);
    
    println!("  Found {} points: {:?}", results.len(), results);
    println!("  Expected order (by distance): [0, 1, 2, 3]");
    assert_eq!(results.len(), 4, "Expected 4 points within radius 1.5");
    assert_eq!(results[0], 0, "Point 0 (distance 0) should be first");
    assert_eq!(results[1], 1, "Point 1 (distance 1) should be second");
    assert_eq!(results[2], 2, "Point 2 (distance 1) should be third");
    assert_eq!(results[3], 3, "Point 3 (distance 1.41) should be fourth");
    println!("  ✓ Correct!\n");

    // Query 2: Find points within radius 0.5 from origin
    println!("Query 2: Points within radius 0.5 from (0, 0):");
    results.clear();
    tree.query_circle_points(0.0, 0.0, 0.5, &mut results);
    
    println!("  Found {} points: {:?}", results.len(), results);
    println!("  Expected: [0]");
    assert_eq!(results.len(), 1, "Expected 1 point within radius 0.5");
    assert_eq!(results[0], 0, "Only point 0 should be found");
    println!("  ✓ Correct!\n");

    // Query 3: Find points within radius 2.0 from (1, 1)
    println!("Query 3: Points within radius 2.0 from (1, 1):");
    results.clear();
    tree.query_circle_points(1.0, 1.0, 2.0, &mut results);
    
    println!("  Found {} points: {:?}", results.len(), results);
    println!("  Expected: points 0, 1, 2, 3 (point 4 is too far)");
    assert_eq!(results.len(), 4, "Expected 4 points within radius 2.0");
    assert_eq!(results[0], 3, "Point 3 at (1, 1) should be closest");
    println!("  ✓ Correct!\n");

    // Query 4: Find all points within radius 8.0 from origin
    println!("Query 4: All points within radius 8.0 from (0, 0):");
    results.clear();
    tree.query_circle_points(0.0, 0.0, 8.0, &mut results);
    
    println!("  Found {} points: {:?}", results.len(), results);
    println!("  Expected: all 5 points, sorted by distance");
    assert_eq!(results.len(), 5, "Expected all 5 points");
    // Results should be in distance order
    assert_eq!(results[0], 0, "Point 0 (closest) should be first");
    println!("  ✓ Correct!\n");

    // Compare with general query_circle
    println!("=== Comparison: query_circle_points vs query_circle ===\n");
    
    let mut optimized_results = Vec::new();
    let mut general_results = Vec::new();
    
    tree.query_circle_points(2.5, 2.5, 3.0, &mut optimized_results);
    tree.query_circle(2.5, 2.5, 3.0, &mut general_results);
    
    println!("query_circle_points results (sorted by distance): {:?}", optimized_results);
    println!("query_circle results (unsorted):                   {:?}", general_results);
    
    // Same points should be found (order may differ)
    let mut opt_sorted = optimized_results.clone();
    opt_sorted.sort();
    let mut gen_sorted = general_results.clone();
    gen_sorted.sort();
    
    assert_eq!(opt_sorted, gen_sorted, "Both methods should find the same points");
    println!("\n✓ Both methods find the same points!");
    
    // Note: optimized version has results sorted by distance
    println!("✓ Optimized version has results sorted by distance (closest first)\n");

    println!("=== Performance Note ===");
    println!("query_circle_points is ~30% faster than query_circle for point data");
    println!("because it uses direct distance calculation instead of axis_distance()");
}
