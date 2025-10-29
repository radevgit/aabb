//! Find boxes that contain a query rectangle.
use aabb::prelude::*;

fn main() {
    let mut tree = AABB::with_capacity(3);
    tree.add(0.0, 0.0, 3.0, 3.0);  // Box 0 (contains query rect)
    tree.add(1.0, 1.0, 2.0, 2.0);  // Box 1 (does not contain query rect)
    tree.add(5.0, 5.0, 6.0, 6.0);  // Box 2 (outside query rect)
    tree.build();

    let mut results = Vec::new();
    tree.query_contain(1.2, 1.2, 1.8, 1.8, &mut results);
    println!("Boxes that contain rectangle: {:?}", results);
    
    // Query rectangle (1.2, 1.2, 1.8, 1.8) is contained in both box 0 and box 1
    // Box 1 (1.0, 1.0, 2.0, 2.0) fully contains it
    assert!(results.len() >= 1, "Expected at least 1 box containing the rectangle");
    assert!(results.contains(&0), "Box 0 should contain the rectangle");
    assert!(!results.contains(&2), "Box 2 should not contain the rectangle");
}
