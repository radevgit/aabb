//! Hilbert curve SVG visualizer
//! Generates an SVG showing a grid with the Hilbert space-filling curve

use std::fs::File;
use std::io::Write;

/// Generate Hilbert curve points for a given order (level)
fn hilbert_curve(order: u32) -> Vec<(f64, f64)> {
    let n = 2u32.pow(order);
    let mut points = Vec::new();

    for i in 0..(n * n) {
        let (x, y) = index_to_xy(i, order);
        points.push((x as f64, y as f64));
    }

    points
}

/// Convert Hilbert curve index to (x, y) coordinates
fn index_to_xy(index: u32, order: u32) -> (u32, u32) {
    let mut x = 0u32;
    let mut y = 0u32;
    let mut s = 1u32;

    let mut i = index;
    while s < (1u32 << order) {
        let rx = 1 & (i >> 1);
        let ry = 1 & (i ^ rx);
        
        if ry == 0 {
            if rx == 1 {
                x = s - 1 - x;
                y = s - 1 - y;
            }
            std::mem::swap(&mut x, &mut y);
        }

        x += rx * s;
        y += ry * s;
        i >>= 2;
        s <<= 1;
    }

    (x, y)
}

fn main() {
    let width = 1280.0;
    let height = 640.0;
    let margin = 50.0;
    let grid_width = width - 2.0 * margin;
    let grid_height = height - 2.0 * margin;

    // Parameter: R-tree level (order)
    // Level 1: order 1 (2x2 grid)
    // Level 2: order 2 (4x4 grid)
    // Level 3: order 3 (8x8 grid)
    // Level 4: order 4 (16x16 grid)
    // Level 5: order 5 (32x32 grid)
    let order = 5u32; // Change this parameter for different levels
    
    let n = 2u32.pow(order) as f64;
    let cell_width = grid_width / n;
    let cell_height = grid_height / n;

    let points = hilbert_curve(order);

    // Convert points to SVG coordinates
    let mut svg_points = Vec::new();
    for (x, y) in &points {
        let svg_x = margin + (x + 0.5) * cell_width;
        let svg_y = height - margin - (y + 0.5) * cell_height;
        svg_points.push((svg_x, svg_y));
    }

    // Generate SVG
    let mut svg = String::new();
    svg.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    svg.push_str(&format!(
        "<svg width=\"{}\" height=\"{}\" xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 {} {}\">\n",
        width as i32, height as i32, width as i32, height as i32
    ));

    svg.push_str("  <defs>\n");
    svg.push_str("    <style>\n");
    svg.push_str("      .grid-line { stroke: #CCCCCC; stroke-width: 1; }\n");
    svg.push_str("      .hilbert-curve { fill: none; stroke: #FF6B35; stroke-width: 2.5; stroke-linecap: round; stroke-linejoin: round; }\n");
    svg.push_str("      .grid-box { fill: none; stroke: #333333; stroke-width: 2; }\n");
    svg.push_str("    </style>\n");
    svg.push_str("  </defs>\n\n");

    // Outer rectangle
    svg.push_str(&format!(
        "  <rect x=\"{}\" y=\"{}\" width=\"{}\" height=\"{}\" class=\"grid-box\"/>\n\n",
        margin as i32, margin as i32, grid_width as i32, grid_height as i32
    ));

    // Grid lines
    svg.push_str("  <g id=\"grid\">\n");

    // Vertical lines
    for i in 1..(n as i32) {
        let x = margin + i as f64 * cell_width;
        svg.push_str(&format!(
            "    <line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\" class=\"grid-line\"/>\n",
            x as i32, margin as i32, x as i32, (height - margin) as i32
        ));
    }

    // Horizontal lines
    for i in 1..(n as i32) {
        let y = margin + i as f64 * cell_height;
        svg.push_str(&format!(
            "    <line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\" class=\"grid-line\"/>\n",
            margin as i32, y as i32, (width - margin) as i32, y as i32
        ));
    }

    svg.push_str("  </g>\n\n");

    // Hilbert curve path
    svg.push_str("  <!-- Hilbert curve -->\n");
    svg.push_str("  <path d=\"M");

    for (i, (x, y)) in svg_points.iter().enumerate() {
        if i == 0 {
            svg.push_str(&format!(" {:.2} {:.2}", x, y));
        } else {
            svg.push_str(&format!(" L {:.2} {:.2}", x, y));
        }
    }

    svg.push_str("\" class=\"hilbert-curve\"/>\n");

    svg.push_str("</svg>\n");

    // Write to file
    let mut file = File::create("hilbert_curve_grid.svg").expect("Failed to create file");
    file.write_all(svg.as_bytes())
        .expect("Failed to write to file");

    println!("✓ Generated: hilbert_curve_grid.svg");
    println!("  - Grid: {}×{} cells", n as i32, n as i32);
    println!("  - Hilbert curve points: {}", svg_points.len());
}
