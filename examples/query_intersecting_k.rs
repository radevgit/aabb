//! Find K first intersecting boxes.
use aabb::prelude::*;

fn main() {
    let mut tree = AABB::with_capacity(3);
    tree.add(0.0, 0.0, 1.0, 1.0);
    tree.add(0.5, 0.5, 1.5, 1.5);
    tree.add(0.8, 0.8, 1.8, 1.8);
    tree.build();

    let mut results = Vec::new();
    tree.query_intersecting_k(0.7, 0.7, 1.3, 1.3, 2, &mut results);
    println!("First 2 intersecting: {:?}", results);
}
