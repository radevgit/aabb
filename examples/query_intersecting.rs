//! Find boxes that intersect a query rectangle.
use aabb::prelude::*;

fn main() {
    let mut tree = AABB::with_capacity(3);
    tree.add(0.0, 0.0, 1.0, 1.0);  // Box 0
    tree.add(2.0, 2.0, 3.0, 3.0);  // Box 1 (outside query)
    tree.add(0.5, 0.5, 1.5, 1.5);  // Box 2
    tree.build();

    let mut results = Vec::new();
    tree.query_intersecting(0.7, 0.7, 1.3, 1.3, &mut results);
    println!("Intersecting: {:?}", results);
    
    // Query rectangle (0.7, 0.7, 1.3, 1.3) intersects boxes 0 and 2, but not box 1
    assert_eq!(results.len(), 2, "Expected 2 intersecting boxes");
    assert!(results.contains(&0), "Box 0 should intersect");
    assert!(results.contains(&2), "Box 2 should intersect");
    assert!(!results.contains(&1), "Box 1 should not intersect");
}
