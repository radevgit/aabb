//! Find boxes intersecting a circular region.
use aabb::prelude::*;

fn main() {
    let mut tree = AABB::with_capacity(3);
    tree.add(0.0, 0.0, 1.0, 1.0);      // Box 0 (within circle)
    tree.add(1.5, 1.5, 2.5, 2.5);      // Box 1 (within circle)
    tree.add(5.0, 5.0, 6.0, 6.0);      // Box 2 (outside circle)
    tree.build();

    let mut results = Vec::new();
    tree.query_circle(1.0, 1.0, 1.5, &mut results);
    println!("In circle: {:?}", results);
    
    // Circle center at (1.0, 1.0) with radius 1.5 intersects boxes 0 and 1, but not box 2
    assert_eq!(results.len(), 2, "Expected 2 boxes in circle");
    assert!(results.contains(&0), "Box 0 should be in circle");
    assert!(results.contains(&1), "Box 1 should be in circle");
    assert!(!results.contains(&2), "Box 2 should not be in circle");
}
