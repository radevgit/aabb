//! Find boxes that intersect with a specific item already in the tree.
//! This is useful for finding all items that collide with a particular item
//! without needing to manually extract its bounding box.
use aabb::prelude::*;

fn main() {
    let mut tree = AABB::with_capacity(4);
    tree.add(0.0, 0.0, 2.0, 2.0);  // Box 0
    tree.add(1.0, 1.0, 3.0, 3.0);  // Box 1 (intersects box 0)
    tree.add(4.0, 4.0, 5.0, 5.0);  // Box 2 (no intersection)
    tree.add(1.5, 1.5, 2.5, 2.5);  // Box 3 (intersects box 0)
    tree.build();

    // Find all boxes that intersect with box 0
    let mut results = Vec::new();
    tree.query_intersecting_id(0, &mut results).unwrap();
    println!("Boxes intersecting with item 0: {:?}", results);
    
    // Box 0 intersects with boxes 1 and 3, but not box 2 (and not itself)
    assert_eq!(results.len(), 2, "Expected 2 intersecting boxes");
    assert!(results.contains(&1), "Box 1 should intersect with box 0");
    assert!(results.contains(&3), "Box 3 should intersect with box 0");
    assert!(!results.contains(&0), "Box 0 should not include itself");
    assert!(!results.contains(&2), "Box 2 should not intersect with box 0");
    
    println!("✓ All assertions passed!");
}
