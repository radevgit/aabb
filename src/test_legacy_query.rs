#[cfg(test)]
mod test_legacy_direction {
    use crate::HilbertRTreeLeg;

    #[test]
    fn test_legacy_query_in_direction() {
        let boxes = vec![
            (10.0, 10.0, 15.0, 15.0),
            (30.0, 30.0, 35.0, 35.0),
            (50.0, 50.0, 55.0, 55.0),
        ];

        let mut tree_leg = HilbertRTreeLeg::new();
        for &(min_x, min_y, max_x, max_y) in &boxes {
            tree_leg.add(min_x, min_y, max_x, max_y);
        }
        tree_leg.build();

        // Check sorted_order (access via public field if available, otherwise we need to infer)
        println!("Boxes in tree (from sorted_order):");
        for (i, &idx) in tree_leg.sorted_order.iter().enumerate() {
            let (min_x, min_y, max_x, max_y) = tree_leg.boxes[idx];
            println!("  Position {}: box index {} at ({:.1}, {:.1})-({:.1}, {:.1})", i, idx, min_x, min_y, max_x, max_y);
        }

        // Calculate swept area manually
        let dir_len_sq = 1.0_f64 * 1.0 + 1.0 * 1.0;
        let dir_len = dir_len_sq.sqrt();
        let norm_dir_x = 1.0 / dir_len;
        let norm_dir_y = 1.0 / dir_len;
        
        let movement_x = norm_dir_x * 50.0;
        let movement_y = norm_dir_y * 50.0;
        
        let swept_min_x = 15.0_f64.min(15.0 + movement_x);
        let swept_max_x = 25.0_f64.max(25.0 + movement_x);
        let swept_min_y = 15.0_f64.min(15.0 + movement_y);
        let swept_max_y = 25.0_f64.max(25.0 + movement_y);
        
        println!("Swept area: ({:.4}, {:.4}) - ({:.4}, {:.4})", swept_min_x, swept_min_y, swept_max_x, swept_max_y);
        
        // Check each box
        for (i, &(min_x, min_y, max_x, max_y)) in boxes.iter().enumerate() {
            let intersects_with_le = min_x <= swept_max_x && max_x >= swept_min_x &&
                                     min_y <= swept_max_y && max_y >= swept_min_y;
            let intersects_with_lt = min_x < swept_max_x && max_x > swept_min_x &&
                                     min_y < swept_max_y && max_y > swept_min_y;
            println!("Box {} ({:.1}, {:.1})-({:.1}, {:.1})", i, min_x, min_y, max_x, max_y);
            println!("  with <=,>= : {}", intersects_with_le);
            println!("  with <,>   : {}", intersects_with_lt);
        }

        // Debug: print the actual check we're doing
        println!("\nManual check using legacy's sorted_order:");
        let mut manual_results = Vec::new();
        for &idx in &tree_leg.sorted_order {
            let (min_x, min_y, max_x, max_y) = tree_leg.boxes[idx];
            if min_x <= swept_max_x && max_x >= swept_min_x &&
               min_y <= swept_max_y && max_y >= swept_min_y {
                println!("  Box {} matches (using <=, >=)", idx);
                manual_results.push(idx);
            } else {
                println!("  Box {} does NOT match (using <=, >=)", idx);
            }
        }
        println!("Manual results with <=,>=: {:?}", manual_results);
        
        // Check what the issue might be - is box 0 outside the rect or something?
        println!("\nBox 0 details:");
        println!("  Box 0: (10,10)-(15,15)");
        println!("  Starting rect: (15,15)-(25,25)");
        println!("  Swept area: ({:.4},{:.4})-({:.4},{:.4})", swept_min_x, swept_min_y, swept_max_x, swept_max_y);
        
        // Check if it could be a distance-based filter (boxes before the start rect?)
        // Box 0 ends at 15,15 which is exactly the start of the rect
        // Boxes 1 and 2 are ahead of the starting position along the direction
        let rect_center_x = (15.0 + 25.0) / 2.0;
        let rect_center_y = (15.0 + 25.0) / 2.0;
        println!("Starting rect center: ({}, {})", rect_center_x, rect_center_y);
        
        for &(min_x, min_y, max_x, max_y) in &boxes {
            let box_center_x = (min_x + max_x) / 2.0;
            let box_center_y = (min_y + max_y) / 2.0;
            let dx = box_center_x - rect_center_x;
            let dy = box_center_y - rect_center_y;
            let dist = (dx*dx + dy*dy).sqrt();
            println!("Box ({},{}) - center ({:.1}, {:.1}), dist={:.4}", min_x, min_y, box_center_x, box_center_y, dist);
        }
        
        let mut results_leg = Vec::new();
        tree_leg.query_in_direction(15.0, 15.0, 25.0, 25.0, 1.0, 1.0, 50.0, &mut results_leg);

        results_leg.sort();
        println!("Legacy query_in_direction results: {:?}", results_leg);
        
        // Don't assert - just print to see what it returns
    }
}
