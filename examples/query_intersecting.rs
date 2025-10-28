//! Find boxes that intersect a query rectangle.
use aabb::prelude::*;

fn main() {
    let mut tree = AABB::with_capacity(3);
    tree.add(0.0, 0.0, 1.0, 1.0);
    tree.add(2.0, 2.0, 3.0, 3.0);
    tree.add(0.5, 0.5, 1.5, 1.5);
    tree.build();

    let mut results = Vec::new();
    tree.query_intersecting(0.7, 0.7, 1.3, 1.3, &mut results);
    println!("Intersecting: {:?}", results);
}
