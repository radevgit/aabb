//! Find boxes intersecting a rectangle's movement path.
use aabb::prelude::*;

fn main() {
    let mut tree = AABB::with_capacity(3);
    tree.add(0.0, 0.0, 1.0, 1.0);    // Box 0 (at start)
    tree.add(2.0, 2.0, 3.0, 3.0);    // Box 1 (in sweep path)
    tree.add(4.0, 4.0, 5.0, 5.0);    // Box 2 (in sweep path)
    tree.build();

    let mut results = Vec::new();
    // Rectangle (0.5, 0.5, 1.5, 1.5) moving diagonally (1.0, 1.0) for distance 4.0
    tree.query_in_direction(0.5, 0.5, 1.5, 1.5, 1.0, 1.0, 4.0, &mut results);
    println!("In direction: {:?}", results);
    
    // Sweep path should intersect all 3 boxes
    assert!(results.len() >= 1, "Expected at least 1 box in sweep path");
    assert!(results.iter().all(|&idx| idx < 3), "All results should be valid indices");
}
