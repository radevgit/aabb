//! Component tests for HilbertRTreeI32
//! 
//! These tests verify individual query methods work correctly with i32 coordinates.
//! Tests cover edge cases, boundary conditions, and basic functionality.

#[cfg(test)]
mod tests {
    use crate::HilbertRTreeI32;

    // ============================================================================
    // BASIC CONSTRUCTION TESTS
    // ============================================================================

    #[test]
    fn test_new_empty_tree() {
        let tree = HilbertRTreeI32::new();
        assert_eq!(tree.len(), 0);
        assert!(tree.is_empty());
    }

    #[test]
    fn test_with_capacity() {
        let tree = HilbertRTreeI32::with_capacity(1000);
        assert_eq!(tree.len(), 0);
        assert!(tree.is_empty());
    }

    #[test]
    fn test_add_single_box() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(0, 0, 10, 10);
        assert_eq!(tree.len(), 1);
        assert!(!tree.is_empty());
    }

    #[test]
    fn test_add_multiple_boxes() {
        let mut tree = HilbertRTreeI32::new();
        for i in 0..100 {
            tree.add(i * 10, i * 10, i * 10 + 5, i * 10 + 5);
        }
        assert_eq!(tree.len(), 100);
    }

    #[test]
    fn test_build_empty_tree() {
        let mut tree = HilbertRTreeI32::new();
        tree.build(); // Should not panic
        assert_eq!(tree.len(), 0);
    }

    #[test]
    fn test_build_single_item() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(0, 0, 10, 10);
        tree.build();
        assert_eq!(tree.len(), 1);
    }

    #[test]
    fn test_build_multiple_items() {
        let mut tree = HilbertRTreeI32::with_capacity(1000);
        for i in 0..50 {
            tree.add(i * 2, i * 2, i * 2 + 1, i * 2 + 1);
        }
        tree.build();
        assert_eq!(tree.len(), 50);
    }

    // ============================================================================
    // QUERY_INTERSECTING TESTS
    // ============================================================================

    #[test]
    fn test_query_intersecting_basic() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(0, 0, 10, 10);
        tree.add(5, 5, 15, 15);
        tree.add(20, 20, 30, 30);
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(0, 0, 10, 10, &mut results);
        assert_eq!(results.len(), 2); // boxes 0 and 1 intersect
    }

    #[test]
    fn test_query_intersecting_no_results() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(0, 0, 10, 10);
        tree.add(20, 20, 30, 30);
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(50, 50, 60, 60, &mut results);
        assert_eq!(results.len(), 0);
    }

    #[test]
    fn test_query_intersecting_touching_edges() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(0, 0, 10, 10);
        tree.add(10, 10, 20, 20); // touches corner of first
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(0, 0, 10, 10, &mut results);
        assert_eq!(results.len(), 2); // both should be included
    }

    #[test]
    fn test_query_intersecting_empty_tree() {
        let tree = HilbertRTreeI32::new();
        let mut results = Vec::new();
        tree.query_intersecting(0, 0, 10, 10, &mut results);
        assert_eq!(results.len(), 0);
    }

    #[test]
    fn test_query_intersecting_large_dataset() {
        let mut tree = HilbertRTreeI32::with_capacity(1000);
        for i in 0..100 {
            tree.add(i * 3, i * 3, i * 3 + 2, i * 3 + 2);
        }
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(50, 50, 100, 100, &mut results);
        assert!(!results.is_empty());
    }

    // ============================================================================
    // QUERY_INTERSECTING_K TESTS
    // ============================================================================

    #[test]
    fn test_query_intersecting_k_basic() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(0, 0, 5, 5);
        tree.add(2, 2, 7, 7);
        tree.add(4, 4, 9, 9);
        tree.add(50, 50, 60, 60);
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting_k(0, 0, 10, 10, 2, &mut results);
        assert!(results.len() <= 2);
    }

    #[test]
    fn test_query_intersecting_k_zero() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(0, 0, 10, 10);
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting_k(0, 0, 10, 10, 0, &mut results);
        assert_eq!(results.len(), 0);
    }

    #[test]
    fn test_query_intersecting_k_k_larger_than_results() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(0, 0, 5, 5);
        tree.add(2, 2, 7, 7);
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting_k(0, 0, 10, 10, 100, &mut results);
        assert_eq!(results.len(), 2); // only 2 boxes available
    }

    #[test]
    fn test_query_intersecting_k_empty_tree() {
        let tree = HilbertRTreeI32::new();
        let mut results = Vec::new();
        tree.query_intersecting_k(0, 0, 10, 10, 5, &mut results);
        assert_eq!(results.len(), 0);
    }

    // ============================================================================
    // QUERY_POINT TESTS
    // ============================================================================

    #[test]
    fn test_query_point_basic() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(0, 0, 10, 10);
        tree.add(5, 5, 15, 15);
        tree.build();

        let mut results = Vec::new();
        tree.query_point(5, 5, &mut results);
        assert_eq!(results.len(), 2); // point (5,5) is in both boxes
    }

    #[test]
    fn test_query_point_no_results() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(0, 0, 10, 10);
        tree.build();

        let mut results = Vec::new();
        tree.query_point(50, 50, &mut results);
        assert_eq!(results.len(), 0);
    }

    #[test]
    fn test_query_point_on_edge() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(0, 0, 10, 10);
        tree.build();

        let mut results = Vec::new();
        tree.query_point(0, 0, &mut results); // corner
        assert_eq!(results.len(), 1);

        results.clear();
        tree.query_point(10, 10, &mut results); // opposite corner
        assert_eq!(results.len(), 1);

        results.clear();
        tree.query_point(0, 5, &mut results); // edge middle
        assert_eq!(results.len(), 1);
    }

    #[test]
    fn test_query_point_outside_bounds() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(0, 0, 10, 10);
        tree.build();

        let mut results = Vec::new();
        tree.query_point(11, 11, &mut results);
        assert_eq!(results.len(), 0);
    }

    #[test]
    fn test_query_point_empty_tree() {
        let tree = HilbertRTreeI32::new();
        let mut results = Vec::new();
        tree.query_point(5, 5, &mut results);
        assert_eq!(results.len(), 0);
    }

    #[test]
    fn test_query_point_negative_coordinates() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(-10, -10, 0, 0);
        tree.build();

        let mut results = Vec::new();
        tree.query_point(-5, -5, &mut results);
        assert_eq!(results.len(), 1);

        results.clear();
        tree.query_point(5, 5, &mut results);
        assert_eq!(results.len(), 0);
    }

    // ============================================================================
    // QUERY_CONTAIN TESTS
    // ============================================================================

    #[test]
    fn test_query_contain_basic() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(0, 0, 20, 20);   // large box
        tree.add(5, 5, 15, 15);   // medium box inside
        tree.add(25, 25, 30, 30); // outside
        tree.build();

        let mut results = Vec::new();
        tree.query_contain(6, 6, 14, 14, &mut results);
        assert_eq!(results.len(), 2); // both large and medium contain the query
    }

    #[test]
    fn test_query_contain_no_results() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(0, 0, 5, 5);
        tree.build();

        let mut results = Vec::new();
        tree.query_contain(0, 0, 10, 10, &mut results); // query larger than box
        assert_eq!(results.len(), 0);
    }

    #[test]
    fn test_query_contain_exact_match() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(0, 0, 10, 10);
        tree.build();

        let mut results = Vec::new();
        tree.query_contain(0, 0, 10, 10, &mut results);
        assert_eq!(results.len(), 1);
    }

    #[test]
    fn test_query_contain_empty_tree() {
        let tree = HilbertRTreeI32::new();
        let mut results = Vec::new();
        tree.query_contain(0, 0, 10, 10, &mut results);
        assert_eq!(results.len(), 0);
    }

    #[test]
    fn test_query_contain_boundary() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(0, 0, 10, 10);
        tree.build();

        let mut results = Vec::new();
        tree.query_contain(0, 0, 9, 9, &mut results); // slightly smaller
        assert_eq!(results.len(), 1);

        results.clear();
        tree.query_contain(1, 1, 10, 10, &mut results); // starts inside but extends to edge
        assert_eq!(results.len(), 1);
    }

    // ============================================================================
    // QUERY_CONTAINED_WITHIN TESTS
    // ============================================================================

    #[test]
    fn test_query_contained_within_basic() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(0, 0, 20, 20);   // large
        tree.add(5, 5, 15, 15);   // medium, inside query
        tree.add(25, 25, 30, 30); // outside
        tree.build();

        let mut results = Vec::new();
        tree.query_contained_within(0, 0, 20, 20, &mut results);
        assert_eq!(results.len(), 2); // both boxes fit within query
    }

    #[test]
    fn test_query_contained_within_no_results() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(0, 0, 20, 20); // too large for query
        tree.build();

        let mut results = Vec::new();
        tree.query_contained_within(5, 5, 15, 15, &mut results);
        assert_eq!(results.len(), 0); // box extends beyond query
    }

    #[test]
    fn test_query_contained_within_exact_match() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(5, 5, 15, 15);
        tree.build();

        let mut results = Vec::new();
        tree.query_contained_within(5, 5, 15, 15, &mut results);
        assert_eq!(results.len(), 1);
    }

    #[test]
    fn test_query_contained_within_empty_tree() {
        let tree = HilbertRTreeI32::new();
        let mut results = Vec::new();
        tree.query_contained_within(0, 0, 10, 10, &mut results);
        assert_eq!(results.len(), 0);
    }

    #[test]
    fn test_query_contained_within_boundary() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(5, 5, 15, 15);
        tree.build();

        let mut results = Vec::new();
        tree.query_contained_within(5, 5, 15, 15, &mut results);
        assert_eq!(results.len(), 1); // exactly contained

        results.clear();
        tree.query_contained_within(4, 4, 16, 16, &mut results);
        assert_eq!(results.len(), 1); // contained with margin

        results.clear();
        tree.query_contained_within(5, 5, 14, 14, &mut results);
        assert_eq!(results.len(), 0); // box extends beyond query
    }

    // ============================================================================
    // COORDINATE RANGE TESTS
    // ============================================================================

    #[test]
    fn test_negative_coordinates() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(-100, -100, -50, -50);
        tree.add(-25, -25, 0, 0);
        tree.add(0, 0, 25, 25);
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(-50, -50, 0, 0, &mut results);
        // Query intersects with box 1 and 2 (box 0 ends at -50 which touches the edge)
        assert!(!results.is_empty());
        assert!(results.contains(&1));
    }

    #[test]
    fn test_large_positive_coordinates() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(1000000, 1000000, 1000100, 1000100);
        tree.add(1000050, 1000050, 1000150, 1000150);
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(1000000, 1000000, 1000100, 1000100, &mut results);
        assert_eq!(results.len(), 2);
    }

    #[test]
    fn test_mixed_positive_negative_coordinates() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(-50, -50, 50, 50);
        tree.add(-100, -100, 100, 100);
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(0, 0, 25, 25, &mut results);
        assert_eq!(results.len(), 2);
    }

    #[test]
    fn test_i32_max_coordinates() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(i32::MAX - 100, i32::MAX - 100, i32::MAX, i32::MAX);
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(i32::MAX - 50, i32::MAX - 50, i32::MAX, i32::MAX, &mut results);
        assert_eq!(results.len(), 1);
    }

    #[test]
    fn test_i32_min_coordinates() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(i32::MIN, i32::MIN, i32::MIN + 100, i32::MIN + 100);
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(i32::MIN, i32::MIN, i32::MIN + 50, i32::MIN + 50, &mut results);
        assert_eq!(results.len(), 1);
    }

    // ============================================================================
    // ZERO-SIZE BOX TESTS
    // ============================================================================

    #[test]
    fn test_zero_width_box() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(5, 0, 5, 10); // zero width (line)
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(4, 0, 6, 10, &mut results);
        assert_eq!(results.len(), 1);
    }

    #[test]
    fn test_zero_height_box() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(0, 5, 10, 5); // zero height (line)
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(0, 4, 10, 6, &mut results);
        assert_eq!(results.len(), 1);
    }

    #[test]
    fn test_point_box() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(5, 5, 5, 5); // zero-size (point)
        tree.build();

        let mut results = Vec::new();
        tree.query_point(5, 5, &mut results);
        assert_eq!(results.len(), 1);

        results.clear();
        tree.query_intersecting(4, 4, 6, 6, &mut results);
        assert_eq!(results.len(), 1);
    }

    // ============================================================================
    // LARGE DATASET TESTS
    // ============================================================================

    #[test]
    fn test_large_dataset_1000_items() {
        let mut tree = HilbertRTreeI32::with_capacity(1000);
        for i in 0..1000 {
            let x = ((i % 50) * 4) as i32;
            let y = ((i / 50) * 4) as i32;
            tree.add(x, y, x + 2, y + 2);
        }
        tree.build();
        assert_eq!(tree.len(), 1000);

        let mut results = Vec::new();
        tree.query_intersecting(0, 0, 100, 100, &mut results);
        assert!(!results.is_empty());
    }

    #[test]
    fn test_large_dataset_performance() {
        let mut tree = HilbertRTreeI32::with_capacity(5000);
        for i in 0..5000 {
            let x = (i * 2) as i32;
            tree.add(x, x, x + 5, x + 5);
        }
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(0, 0, 1000, 1000, &mut results);
        assert!(!results.is_empty());
    }

    // ============================================================================
    // GRID LAYOUT TESTS
    // ============================================================================

    #[test]
    fn test_regular_grid_layout() {
        let mut tree = HilbertRTreeI32::new();
        let grid_size = 10;
        let cell_size = 10;

        for x in 0..grid_size {
            for y in 0..grid_size {
                let min_x = (x * cell_size) as i32;
                let min_y = (y * cell_size) as i32;
                tree.add(min_x, min_y, min_x + cell_size as i32 - 1, min_y + cell_size as i32 - 1);
            }
        }
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(0, 0, 50, 50, &mut results);
        assert!(!results.is_empty());
    }

    #[test]
    fn test_scattered_boxes() {
        let mut tree = HilbertRTreeI32::new();
        let positions = vec![
            (0, 0, 5, 5),
            (50, 50, 55, 55),
            (100, 100, 105, 105),
            (150, 150, 155, 155),
        ];

        for (min_x, min_y, max_x, max_y) in positions {
            tree.add(min_x, min_y, max_x, max_y);
        }
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(40, 40, 60, 60, &mut results);
        assert_eq!(results.len(), 1); // only one box in this range
    }

    // ============================================================================
    // RESULT VECTOR REUSE TESTS
    // ============================================================================

    #[test]
    fn test_result_vector_cleared_between_queries() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(0, 0, 10, 10);
        tree.add(50, 50, 60, 60);
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(0, 0, 10, 10, &mut results);
        assert_eq!(results.len(), 1);

        tree.query_intersecting(50, 50, 60, 60, &mut results);
        assert_eq!(results.len(), 1); // should be cleared before second query
    }

    #[test]
    fn test_reusing_results_vector() {
        let mut tree = HilbertRTreeI32::new();
        for i in 0..10 {
            tree.add(i * 10, i * 10, i * 10 + 5, i * 10 + 5);
        }
        tree.build();

        let mut results = Vec::new();
        for i in 0..10 {
            tree.query_intersecting(i * 10, i * 10, i * 10 + 20, i * 10 + 20, &mut results);
            assert!(!results.is_empty());
        }
    }

    // ============================================================================
    // DEFAULT TRAIT TESTS
    // ============================================================================

    #[test]
    fn test_default_trait() {
        let tree = HilbertRTreeI32::default();
        assert_eq!(tree.len(), 0);
        assert!(tree.is_empty());
    }

    // ============================================================================
    // CLONE TRAIT TESTS
    // ============================================================================

    #[test]
    fn test_clone_empty_tree() {
        let tree = HilbertRTreeI32::new();
        let cloned = tree.clone();
        assert_eq!(cloned.len(), 0);
    }

    #[test]
    fn test_clone_built_tree() {
        let mut tree = HilbertRTreeI32::new();
        tree.add(0, 0, 10, 10);
        tree.add(5, 5, 15, 15);
        tree.build();

        let cloned = tree.clone();
        let mut results = Vec::new();
        cloned.query_intersecting(0, 0, 10, 10, &mut results);
        assert_eq!(results.len(), 2);
    }
}
