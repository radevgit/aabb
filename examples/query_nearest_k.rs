//! Find K nearest boxes to a point.
use aabb::prelude::*;

fn main() {
    let mut tree = AABB::with_capacity(4);
    tree.add(0.0, 0.0, 1.0, 1.0);    // Box 0 (2nd nearest to (2.5, 2.5))
    tree.add(2.0, 2.0, 3.0, 3.0);    // Box 1 (nearest to (2.5, 2.5))
    tree.add(4.0, 4.0, 5.0, 5.0);    // Box 2
    tree.add(6.0, 6.0, 7.0, 7.0);    // Box 3
    tree.build();

    let mut results = Vec::new();
    tree.query_nearest_k(2.5, 2.5, 2, &mut results);
    println!("2 nearest boxes: {:?}", results);
    
    // Query point (2.5, 2.5) - 2 nearest should be boxes 1 and 0
    assert_eq!(results.len(), 2, "Expected 2 nearest boxes");
    assert!(results.contains(&1), "Box 1 should be nearest");
    assert!(results.contains(&0), "Box 0 should be second nearest");
}
