//! Find boxes that contain a point.
use aabb::prelude::*;

fn main() {
    let mut tree = AABB::with_capacity(3);
    tree.add(0.0, 0.0, 2.0, 2.0);  // Box 0 (contains point)
    tree.add(1.0, 1.0, 3.0, 3.0);  // Box 1 (contains point)
    tree.add(5.0, 5.0, 6.0, 6.0);  // Box 2 (does not contain point)
    tree.build();

    let mut results = Vec::new();
    tree.query_point(1.5, 1.5, &mut results);
    println!("Contains point (1.5, 1.5): {:?}", results);
    
    // Point (1.5, 1.5) is contained in boxes 0 and 1, but not box 2
    assert_eq!(results.len(), 2, "Expected 2 boxes containing the point");
    assert!(results.contains(&0), "Box 0 should contain point");
    assert!(results.contains(&1), "Box 1 should contain point");
    assert!(!results.contains(&2), "Box 2 should not contain point");
}
