//! Find K nearest boxes intersecting a rectangle's movement path.
use aabb::prelude::*;

fn main() {
    let mut tree = AABB::with_capacity(4);
    tree.add(1.0, 1.0, 2.0, 2.0);
    tree.add(2.5, 2.5, 3.5, 3.5);
    tree.add(4.0, 4.0, 5.0, 5.0);
    tree.add(6.0, 6.0, 7.0, 7.0);
    tree.build();

    let mut results = Vec::new();
    tree.query_in_direction_k(0.5, 0.5, 1.5, 1.5, 1.0, 1.0, 2, 8.0, &mut results);
    println!("2 nearest in direction: {:?}", results);
}
