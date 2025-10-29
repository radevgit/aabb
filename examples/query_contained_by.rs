//! Find boxes contained within a query rectangle.
use aabb::prelude::*;

fn main() {
    let mut tree = AABB::with_capacity(3);
    tree.add(1.0, 1.0, 2.0, 2.0);      // Box 0 (contained in query)
    tree.add(1.5, 1.5, 1.8, 1.8);      // Box 1 (contained in query)
    tree.add(5.0, 5.0, 6.0, 6.0);      // Box 2 (outside query)
    tree.build();

    let mut results = Vec::new();
    tree.query_contained_within(0.5, 0.5, 3.0, 3.0, &mut results);
    println!("Boxes contained within rectangle: {:?}", results);
    
    // Query rectangle (0.5, 0.5, 3.0, 3.0) contains boxes 0 and 1, but not box 2
    assert_eq!(results.len(), 2, "Expected 2 boxes contained within rectangle");
    assert!(results.contains(&0), "Box 0 should be contained");
    assert!(results.contains(&1), "Box 1 should be contained");
    assert!(!results.contains(&2), "Box 2 should not be contained");
}
