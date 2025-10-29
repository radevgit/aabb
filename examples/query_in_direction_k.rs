//! Find K nearest boxes intersecting a rectangle's movement path.
use aabb::prelude::*;

fn main() {
    let mut tree = AABB::with_capacity(4);
    tree.add(1.0, 1.0, 2.0, 2.0);      // Box 0
    tree.add(2.5, 2.5, 3.5, 3.5);      // Box 1 (1st in direction)
    tree.add(4.0, 4.0, 5.0, 5.0);      // Box 2 (2nd in direction)
    tree.add(6.0, 6.0, 7.0, 7.0);      // Box 3
    tree.build();

    let mut results = Vec::new();
    // Rectangle (0.5, 0.5, 1.5, 1.5) moving diagonally (1.0, 1.0) for distance 8.0, find 2 nearest
    tree.query_in_direction_k(0.5, 0.5, 1.5, 1.5, 1.0, 1.0, 2, 8.0, &mut results);
    println!("2 nearest in direction: {:?}", results);
    
    // Should return at most k=2 boxes
    assert!(results.len() <= 2, "Expected at most 2 boxes (k=2)");
    assert!(results.iter().all(|&idx| idx < 4), "All results should be valid indices");
}
