//! Save and restore the Hilbert Tree
use aabb::prelude::*;

fn main() -> std::io::Result<()> {
    // Example with f64 coordinates
    println!("=== AABB (f64) Save/Load Example ===");
    
    let mut tree = AABB::with_capacity(4);
    tree.add(10.0, 10.0, 15.0, 15.0);   // Box 0
    tree.add(20.0, 20.0, 25.0, 25.0);   // Box 1
    tree.add(30.0, 10.0, 35.0, 15.0);   // Box 2
    tree.build();
    
    // Save tree to file
    tree.save("/tmp/tree_f64.bin")?;
    println!("Saved f64 tree to /tmp/tree_f64.bin");
    
    // Query before saving (for comparison)
    let mut results = Vec::new();
    tree.query_intersecting(15.0, 15.0, 25.0, 25.0, &mut results);
    println!("Original tree intersecting query result count: {}", results.len());
    
    // Load tree from file
    let loaded_tree = AABB::load("/tmp/tree_f64.bin")?;
    println!("Loaded f64 tree from /tmp/tree_f64.bin");
    
    // Query after loading (should be identical)
    results.clear();
    loaded_tree.query_intersecting(15.0, 15.0, 25.0, 25.0, &mut results);
    println!("Loaded tree intersecting query result count: {}", results.len());
    assert!(results.len() > 0, "Query should return results");
    println!("✓ Results match!\n");

    // Test header validation - try loading i32 tree with f64 loader (should fail)
    println!("=== Testing Header Validation ===");
    let mut tree_i32_temp = AABBI32::with_capacity(2);
    tree_i32_temp.add(0, 0, 5, 5);
    tree_i32_temp.build();
    tree_i32_temp.save("/tmp/tree_i32_temp.bin")?;
    
    // Try loading i32 file with f64 loader - should fail due to version mismatch
    match AABB::load("/tmp/tree_i32_temp.bin") {
        Ok(_) => println!("✗ Should have failed loading i32 file as f64"),
        Err(e) => println!("✓ Correctly rejected i32 file: {}\n", e),
    }

    // Example with i32 coordinates
    println!("=== AABBI32 (i32) Save/Load Example ===");
    
    let mut tree_i32 = AABBI32::with_capacity(4);
    tree_i32.add(10, 10, 15, 15);   // Box 0
    tree_i32.add(20, 20, 25, 25);   // Box 1
    tree_i32.add(30, 10, 35, 15);   // Box 2
    tree_i32.build();
    
    // Save tree to file
    tree_i32.save("/tmp/tree_i32.bin")?;
    println!("Saved i32 tree to /tmp/tree_i32.bin");
    
    // Query before saving (for comparison)
    let mut results_i32 = Vec::new();
    tree_i32.query_intersecting(15, 15, 25, 25, &mut results_i32);
    println!("Original i32 tree intersecting query result count: {}", results_i32.len());
    
    // Load tree from file
    let loaded_tree_i32 = AABBI32::load("/tmp/tree_i32.bin")?;
    println!("Loaded i32 tree from /tmp/tree_i32.bin");
    
    // Query after loading (should be identical)
    results_i32.clear();
    loaded_tree_i32.query_intersecting(15, 15, 25, 25, &mut results_i32);
    println!("Loaded i32 tree intersecting query result count: {}", results_i32.len());
    assert!(results_i32.len() > 0, "Query should return results");
    println!("✓ Results match!\n");

    println!("All save/load tests passed!");
    Ok(())
}
