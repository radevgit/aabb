//! Find boxes intersecting a circular region.
use aabb::prelude::*;

fn main() {
    let mut tree = AABB::with_capacity(3);
    tree.add(0.0, 0.0, 1.0, 1.0);
    tree.add(1.5, 1.5, 2.5, 2.5);
    tree.add(5.0, 5.0, 6.0, 6.0);
    tree.build();

    let mut results = Vec::new();
    tree.query_circle(1.0, 1.0, 1.5, &mut results);
    println!("In circle: {:?}", results);
}
