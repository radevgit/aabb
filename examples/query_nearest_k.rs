//! Find K nearest boxes to a point.
use aabb::prelude::*;

fn main() {
    let mut tree = AABB::with_capacity(4);
    tree.add(0.0, 0.0, 1.0, 1.0);
    tree.add(2.0, 2.0, 3.0, 3.0);
    tree.add(4.0, 4.0, 5.0, 5.0);
    tree.add(6.0, 6.0, 7.0, 7.0);
    tree.build();

    let mut results = Vec::new();
    tree.query_nearest_k(2.5, 2.5, 2, &mut results);
    println!("2 nearest boxes: {:?}", results);
}
