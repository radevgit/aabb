//! Benchmark for dynamic usage: rebuilding, repeated `build()` calls and `get()`.
//!
//! The other benchmarks only measure a single `build()` followed by queries.
//! This one covers what happens when the index is used as if the data changes:
//!
//! 1. Fresh tree every frame (the supported pattern) - cost and correctness.
//! 2. Calling `build()` again on an unchanged tree.
//! 3. Adding items to an existing tree and calling `build()` again every frame.
//! 4. `get(id)` cost for a single lookup and for a lookup of every id.
//! 5. Reusing one tree with `clear()` and refilling it every frame.
//!
//! Every scenario also reports correctness: the number of wrong ids found by
//! `get()` in an evenly spaced sample, and whether a fixed query returns exactly
//! the ids a brute-force scan finds. Timing rows of scenarios 2 and 3 are only
//! meaningful for comparison once their correctness rows report 0 wrong ids.
//!
//! Run: `cargo bench --bench dynamic_usage_bench`

use aabb::HilbertRTree;
use rand::Rng;
use rand::SeedableRng;
use std::hint::black_box;
use std::time::{Duration, Instant};

type Rect = (f64, f64, f64, f64);

const SEED: u64 = 95756739;
const BASE_ITEMS: usize = 100_000;
const FRAMES: usize = 20;
const NEW_PER_FRAME: usize = 1_000;
const SAMPLE: usize = 1_000;
const QUERY: Rect = (20.0, 20.0, 40.0, 40.0);

/// Random box in a 100x100 space, size up to 1 (same as the other benchmarks).
fn random_box<R: Rng>(rng: &mut R) -> Rect {
    let min_x = rng.random_range(0.0..99.0);
    let min_y = rng.random_range(0.0..99.0);
    (
        min_x,
        min_y,
        min_x + rng.random_range(0.0..1.0),
        min_y + rng.random_range(0.0..1.0),
    )
}

fn random_boxes<R: Rng>(rng: &mut R, n: usize) -> Vec<Rect> {
    (0..n).map(|_| random_box(rng)).collect()
}

fn ms(d: Duration) -> f64 {
    d.as_secs_f64() * 1000.0
}

/// Number of wrong ids in an evenly spaced sample: `get(id)` must return the box added as `id`.
fn wrong_ids(tree: &HilbertRTree, boxes: &[Rect], sample: usize) -> usize {
    let step = (boxes.len() / sample).max(1);
    (0..boxes.len())
        .step_by(step)
        .filter(|&id| tree.get(id) != Some(boxes[id]))
        .count()
}

/// True if the query returns exactly the ids a brute-force scan finds.
fn query_matches_brute_force(tree: &HilbertRTree, boxes: &[Rect]) -> bool {
    let mut got = Vec::new();
    tree.query_intersecting(QUERY.0, QUERY.1, QUERY.2, QUERY.3, &mut got);
    got.sort_unstable();
    let expected: Vec<usize> = boxes
        .iter()
        .enumerate()
        .filter(|(_, b)| b.0 <= QUERY.2 && b.2 >= QUERY.0 && b.1 <= QUERY.3 && b.3 >= QUERY.1)
        .map(|(i, _)| i)
        .collect();
    got == expected
}

fn report_correctness(tree: &HilbertRTree, boxes: &[Rect]) {
    println!(
        "  correctness: wrong ids {}/{} sampled, query ids match brute force: {}",
        wrong_ids(tree, boxes, SAMPLE),
        SAMPLE.min(boxes.len()),
        query_matches_brute_force(tree, boxes)
    );
}

fn build_tree(boxes: &[Rect]) -> HilbertRTree {
    let mut tree = HilbertRTree::with_capacity(boxes.len());
    for b in boxes {
        tree.add(b.0, b.1, b.2, b.3);
    }
    tree.build();
    tree
}

/// 1. New tree every frame: the supported way to handle changing data.
fn bench_fresh_tree_per_frame(boxes: &[Rect]) {
    println!("\n[1] fresh tree per frame ({} items, {} frames)", boxes.len(), FRAMES);
    let mut total = Duration::ZERO;
    let mut last = HilbertRTree::new();
    for _ in 0..FRAMES {
        let start = Instant::now();
        last = build_tree(boxes);
        total += start.elapsed();
    }
    println!("  add+build per frame: {:.2}ms", ms(total) / FRAMES as f64);
    report_correctness(&last, boxes);
}

/// 2. Calling build() again on an unchanged tree.
fn bench_rebuild_unchanged(boxes: &[Rect]) {
    println!("\n[2] build() called again on unchanged tree ({} items, {} extra builds)", boxes.len(), FRAMES);
    let mut tree = build_tree(boxes);
    let mut total = Duration::ZERO;
    for _ in 0..FRAMES {
        let start = Instant::now();
        tree.build();
        total += start.elapsed();
    }
    println!("  extra build() per call: {:.2}ms", ms(total) / FRAMES as f64);
    report_correctness(&tree, boxes);
}

/// 3. Same tree: add new items and call build() again every frame.
fn bench_add_and_rebuild(boxes: &[Rect], rng: &mut rand::rngs::StdRng) {
    println!(
        "\n[3] add {} items + build() on the same tree, {} frames (start {} items)",
        NEW_PER_FRAME, FRAMES, boxes.len()
    );
    let mut all = boxes.to_vec();
    let mut tree = build_tree(&all);
    let mut total = Duration::ZERO;
    for _ in 0..FRAMES {
        let start = Instant::now();
        for _ in 0..NEW_PER_FRAME {
            let b = random_box(rng);
            all.push(b);
            tree.add(b.0, b.1, b.2, b.3);
        }
        tree.build();
        total += start.elapsed();
    }
    println!("  add+build per frame: {:.2}ms", ms(total) / FRAMES as f64);
    println!("  final items: {}", tree.len());
    report_correctness(&tree, &all);
}

/// 5. Reuse one tree: clear() and refill every frame (keeps the allocated buffer).
fn bench_clear_and_refill(boxes: &[Rect]) {
    println!("\n[5] clear() + refill + build on one reused tree ({} items, {} frames)", boxes.len(), FRAMES);
    let mut tree = HilbertRTree::with_capacity(boxes.len());
    let mut total = Duration::ZERO;
    for _ in 0..FRAMES {
        let start = Instant::now();
        tree.clear();
        for b in boxes {
            tree.add(b.0, b.1, b.2, b.3);
        }
        tree.build();
        total += start.elapsed();
    }
    println!("  clear+add+build per frame: {:.2}ms", ms(total) / FRAMES as f64);
    report_correctness(&tree, boxes);
}

/// 4. Cost of get(id) after build.
fn bench_get(rng: &mut rand::rngs::StdRng) {
    println!("\n[4] get(id) cost");
    for n in [10_000usize, 100_000, 1_000_000] {
        let boxes = random_boxes(rng, n);
        let tree = build_tree(&boxes);
        let step = n / SAMPLE;
        let start = Instant::now();
        for id in (0..n).step_by(step) {
            black_box(tree.get(id));
        }
        let per_call_us = ms(start.elapsed()) * 1000.0 / SAMPLE as f64;
        println!("  n={:>9}: {} get() calls: {:.2}us/call", n, SAMPLE, per_call_us);
    }
    // Typical caller pattern: look up every id once.
    let n = 10_000;
    let boxes = random_boxes(rng, n);
    let tree = build_tree(&boxes);
    let start = Instant::now();
    for id in 0..n {
        black_box(tree.get(id));
    }
    println!("  n={:>9}: get() for every id: {:.2}ms", n, ms(start.elapsed()));
}

fn main() {
    println!("AABB Dynamic Usage Benchmark");
    println!("============================");

    let mut rng = rand::rngs::StdRng::seed_from_u64(SEED);
    let boxes = random_boxes(&mut rng, BASE_ITEMS);

    bench_fresh_tree_per_frame(&boxes);
    bench_rebuild_unchanged(&boxes);
    bench_add_and_rebuild(&boxes, &mut rng);
    bench_get(&mut rng);
    bench_clear_and_refill(&boxes);
}
