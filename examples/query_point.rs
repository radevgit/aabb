//! Find boxes that contain a point.
use aabb::prelude::*;

fn main() {
    let mut tree = AABB::with_capacity(3);
    tree.add(0.0, 0.0, 2.0, 2.0);
    tree.add(1.0, 1.0, 3.0, 3.0);
    tree.add(5.0, 5.0, 6.0, 6.0);
    tree.build();

    let mut results = Vec::new();
    tree.query_point(1.5, 1.5, &mut results);
    println!("Contains point (1.5, 1.5): {:?}", results);
}
