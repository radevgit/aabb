//! Find boxes contained within a query rectangle.
use aabb::prelude::*;

fn main() {
    let mut tree = AABB::with_capacity(3);
    tree.add(1.0, 1.0, 2.0, 2.0);
    tree.add(1.5, 1.5, 1.8, 1.8);
    tree.add(5.0, 5.0, 6.0, 6.0);
    tree.build();

    let mut results = Vec::new();
    tree.query_contained_within(0.5, 0.5, 3.0, 3.0, &mut results);
    println!("Boxes contained within rectangle: {:?}", results);
}
