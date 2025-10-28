//! Find the nearest box to a point.
use aabb::prelude::*;

fn main() {
    let mut tree = AABB::with_capacity(3);
    tree.add(0.0, 0.0, 1.0, 1.0);
    tree.add(3.0, 3.0, 4.0, 4.0);
    tree.add(5.0, 5.0, 6.0, 6.0);
    tree.build();

    if let Some(idx) = tree.query_nearest(2.0, 2.0) {
        println!("Nearest box: {}", idx);
    }
}
