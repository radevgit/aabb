//! Find boxes intersecting a rectangle's movement path.
use aabb::prelude::*;

fn main() {
    let mut tree = AABB::with_capacity(3);
    tree.add(0.0, 0.0, 1.0, 1.0);
    tree.add(2.0, 2.0, 3.0, 3.0);
    tree.add(4.0, 4.0, 5.0, 5.0);
    tree.build();

    let mut results = Vec::new();
    tree.query_in_direction(0.5, 0.5, 1.5, 1.5, 1.0, 1.0, 4.0, &mut results);
    println!("In direction: {:?}", results);
}
