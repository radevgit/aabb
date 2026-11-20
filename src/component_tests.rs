//! Component tests for HilbertRTree - testing each method individually
//! This file provides granular test coverage to identify specific bugs

#[cfg(test)]
mod tests {
    use crate::HilbertRTree;

    // ============================================================================
    // BASIC INITIALIZATION TESTS
    // ============================================================================

    #[test]
    fn test_new_tree() {
        let tree = HilbertRTree::new();
        assert_eq!(tree.num_items, 0, "New tree should be empty");
        assert_eq!(tree.node_size, 16, "Default node size should be 16");
    }

    #[test]
    fn test_with_capacity() {
        let tree = HilbertRTree::with_capacity(1000);
        assert_eq!(tree.num_items, 0, "New tree with capacity should be empty");
        assert_eq!(tree.node_size, 16, "Node size should still be 16");
    }

    // ============================================================================
    // ADD OPERATION TESTS
    // ============================================================================

    #[test]
    fn test_add_single_box() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        assert_eq!(tree.num_items, 1);
    }

    #[test]
    fn test_add_multiple_boxes() {
        let mut tree = HilbertRTree::new();
        for i in 0..10 {
            tree.add(i as f64, i as f64, (i + 1) as f64, (i + 1) as f64);
        }
        assert_eq!(tree.num_items, 10);
    }

    #[test]
    fn test_add_duplicate_coordinates() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.add(10.0, 10.0, 20.0, 20.0);
        assert_eq!(tree.num_items, 2);
    }

    #[test]
    fn test_add_zero_size_box() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 10.0, 10.0);
        assert_eq!(tree.num_items, 1);
    }

    #[test]
    fn test_add_negative_coordinates() {
        let mut tree = HilbertRTree::new();
        tree.add(-100.0, -100.0, -50.0, -50.0);
        assert_eq!(tree.num_items, 1);
    }

    // ============================================================================
    // BUILD OPERATION TESTS
    // ============================================================================

    #[test]
    fn test_build_empty_tree() {
        let mut tree = HilbertRTree::new();
        tree.build();
        assert_eq!(tree.num_items, 0);
    }

    #[test]
    fn test_build_single_item() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.build();
        assert_eq!(tree.num_items, 1);
        if tree.level_bounds.is_empty() {
            panic!("Should have level bounds");
        }
    }

    #[test]
    fn test_build_multiple_items_single_level() {
        let mut tree = HilbertRTree::new();
        for i in 0..5 {
            tree.add(i as f64, i as f64, (i + 1) as f64, (i + 1) as f64);
        }
        tree.build();
        assert_eq!(tree.num_items, 5);
        // 5 items < 16 (node_size), so should have 2 levels (items + root)
        assert_eq!(tree.level_bounds.len(), 2);
        // First level ends at 5 (5 items), second level ends at 6 (5 items + 1 root)
        assert_eq!(tree.level_bounds[0], 5);
        assert_eq!(tree.level_bounds[1], 6);
    }

    #[test]
    fn test_build_multiple_items_multiple_levels() {
        let mut tree = HilbertRTree::new();
        for i in 0..100 {
            tree.add(i as f64, i as f64, (i + 1) as f64, (i + 1) as f64);
        }
        tree.build();
        assert_eq!(tree.num_items, 100);
        // 100 items > 16, so should have multiple levels
        assert!(tree.level_bounds.len() > 1, "Should have multiple hierarchy levels");
    }

    // ============================================================================
    // BOUNDS COMPUTATION TESTS
    // ============================================================================

    #[test]
    fn test_bounds_single_item() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 20.0, 30.0, 40.0);
        tree.build();

        assert_eq!(tree.bounds.min_x, 10.0);
        assert_eq!(tree.bounds.min_y, 20.0);
        assert_eq!(tree.bounds.max_x, 30.0);
        assert_eq!(tree.bounds.max_y, 40.0);
    }

    #[test]
    fn test_bounds_multiple_items() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.add(5.0, 5.0, 25.0, 25.0);
        tree.add(0.0, 0.0, 30.0, 30.0);
        tree.build();

        assert_eq!(tree.bounds.min_x, 0.0);
        assert_eq!(tree.bounds.min_y, 0.0);
        assert_eq!(tree.bounds.max_x, 30.0);
        assert_eq!(tree.bounds.max_y, 30.0);
    }

    #[test]
    fn test_bounds_with_negative_coordinates() {
        let mut tree = HilbertRTree::new();
        tree.add(-10.0, -10.0, 10.0, 10.0);
        tree.add(-20.0, -5.0, 5.0, 15.0);
        tree.build();

        assert_eq!(tree.bounds.min_x, -20.0);
        assert_eq!(tree.bounds.min_y, -10.0);
        assert_eq!(tree.bounds.max_x, 10.0);
        assert_eq!(tree.bounds.max_y, 15.0);
    }

    // ============================================================================
    // GET_BOX TESTS
    // ============================================================================

    #[test]
    fn test_get_box_single_item() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 20.0, 30.0, 40.0);
        tree.build();

        let box_data = tree.get_box(0);
        assert_eq!(box_data.min_x, 10.0);
        assert_eq!(box_data.min_y, 20.0);
        assert_eq!(box_data.max_x, 30.0);
        assert_eq!(box_data.max_y, 40.0);
    }

    #[test]
    fn test_get_box_multiple_items() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.add(30.0, 30.0, 40.0, 40.0);
        tree.add(50.0, 50.0, 60.0, 60.0);
        tree.build();

        let box0 = tree.get_box(0);
        let box1 = tree.get_box(1);
        let box2 = tree.get_box(2);

        // After Hilbert sort, boxes may be reordered, but all should be present
        assert!(
            (box0.min_x == 10.0 || box1.min_x == 10.0 || box2.min_x == 10.0),
            "Box (10-20) should be present"
        );
        assert!(
            (box0.min_x == 30.0 || box1.min_x == 30.0 || box2.min_x == 30.0),
            "Box (30-40) should be present"
        );
        assert!(
            (box0.min_x == 50.0 || box1.min_x == 50.0 || box2.min_x == 50.0),
            "Box (50-60) should be present"
        );
    }

    // ============================================================================
    // GET_INDEX TESTS
    // ============================================================================

    #[test]
    fn test_get_index_single_item() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.build();

        let idx = tree.get_index(0);
        assert_eq!(idx, 0, "First item should have original index 0");
    }

    #[test]
    fn test_get_index_after_sort() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0); // item 0
        tree.add(30.0, 30.0, 40.0, 40.0); // item 1
        tree.add(50.0, 50.0, 60.0, 60.0); // item 2
        tree.build();

        // After Hilbert sort, get_index should return original indices
        let mut indices = Vec::new();
        for i in 0..3 {
            indices.push(tree.get_index(i));
        }

        // Should contain 0, 1, 2 in some order
        indices.sort();
        assert_eq!(indices, vec![0, 1, 2], "Should have all original indices");
    }

    // ============================================================================
    // QUERY_INTERSECTING TESTS - BASIC
    // ============================================================================

    #[test]
    fn test_query_intersecting_empty_tree() {
        let tree = HilbertRTree::new();
        let mut results = Vec::new();
        tree.query_intersecting(0.0, 0.0, 10.0, 10.0, &mut results);
        assert_eq!(results.len(), 0);
    }

    #[test]
    fn test_query_intersecting_empty_result() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(50.0, 50.0, 60.0, 60.0, &mut results);
        assert_eq!(results.len(), 0, "Query should find no intersections");
    }

    #[test]
    fn test_query_intersecting_exact_match() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(10.0, 10.0, 20.0, 20.0, &mut results);
        assert_eq!(results.len(), 1, "Should find exact match");
        assert_eq!(results[0], 0, "Should return correct index");
    }

    #[test]
    fn test_query_intersecting_partial_overlap() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(15.0, 15.0, 25.0, 25.0, &mut results);
        assert_eq!(results.len(), 1, "Should find partial overlap");
    }

    #[test]
    fn test_query_intersecting_contained() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(12.0, 12.0, 18.0, 18.0, &mut results);
        assert_eq!(results.len(), 1, "Should find contained box");
    }

    #[test]
    fn test_query_intersecting_contains() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(5.0, 5.0, 25.0, 25.0, &mut results);
        assert_eq!(results.len(), 1, "Query should contain box");
    }

    // ============================================================================
    // QUERY_INTERSECTING TESTS - MULTIPLE ITEMS
    // ============================================================================

    #[test]
    fn test_query_intersecting_two_items_both_match() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0); // index 0
        tree.add(15.0, 15.0, 25.0, 25.0); // index 1
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(12.0, 12.0, 22.0, 22.0, &mut results);
        assert_eq!(results.len(), 2, "Should find both items");
        results.sort();
        assert_eq!(results, vec![0, 1], "Should return correct indices");
    }

    #[test]
    fn test_query_intersecting_two_items_one_match() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0); // index 0
        tree.add(30.0, 30.0, 40.0, 40.0); // index 1
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(12.0, 12.0, 22.0, 22.0, &mut results);
        assert_eq!(results.len(), 1, "Should find only first item");
        assert_eq!(results[0], 0);
    }

    #[test]
    fn test_query_intersecting_three_items_mixed() {
        let mut tree = HilbertRTree::new();
        tree.add(0.0, 0.0, 10.0, 10.0);   // 0 - matches
        tree.add(20.0, 20.0, 30.0, 30.0); // 1 - no match
        tree.add(5.0, 5.0, 15.0, 15.0);   // 2 - matches
        tree.add(40.0, 40.0, 50.0, 50.0); // 3 - no match
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(2.0, 2.0, 12.0, 12.0, &mut results);
        results.sort();

        assert_eq!(results.len(), 2, "Should find 2 matches");
        assert_eq!(results, vec![0, 2], "Should find items 0 and 2");
    }

    #[test]
    fn test_query_intersecting_no_duplicates() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.add(15.0, 15.0, 25.0, 25.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(12.0, 12.0, 22.0, 22.0, &mut results);

        let original_len = results.len();
        results.sort();
        results.dedup();
        assert_eq!(
            results.len(), original_len,
            "Results should have no duplicates"
        );
    }

    // ============================================================================
    // QUERY_INTERSECTING TESTS - LARGE DATASET
    // ============================================================================

    #[test]
    fn test_query_intersecting_large_dataset() {
        let mut tree = HilbertRTree::with_capacity(100);
        for i in 0..100 {
            tree.add(
                (i * 2) as f64,
                (i * 2) as f64,
                (i * 2 + 1) as f64,
                (i * 2 + 1) as f64,
            );
        }
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(50.0, 50.0, 70.0, 70.0, &mut results);

        // Should find some items in this range
        assert!(!results.is_empty(), "Should find items");

        // All results should be valid indices
        for idx in &results {
            assert!(*idx < 100, "Result index should be valid");
        }
    }

    // ============================================================================
    // QUERY_NEAREST_K TESTS
    // ============================================================================

    #[test]
    fn test_query_nearest_k_empty_tree() {
        let tree = HilbertRTree::new();
        let mut results = Vec::new();
        tree.query_nearest_k(0.0, 0.0, 5, &mut results);
        assert_eq!(results.len(), 0);
    }

    #[test]
    fn test_query_nearest_k_single_item() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_nearest_k(15.0, 15.0, 5, &mut results);
        assert_eq!(results.len(), 1, "Should return 1 result");
        assert_eq!(results[0], 0);
    }

    #[test]
    fn test_query_nearest_k_k_less_than_items() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0); // 0
        tree.add(50.0, 50.0, 60.0, 60.0); // 1
        tree.add(30.0, 30.0, 40.0, 40.0); // 2
        tree.build();

        let mut results = Vec::new();
        tree.query_nearest_k(15.0, 15.0, 1, &mut results);
        assert_eq!(results.len(), 1, "K=1 should return exactly 1");
        assert_eq!(results[0], 0, "Nearest should be item 0");
    }

    #[test]
    fn test_query_nearest_k_k_equals_items() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.add(50.0, 50.0, 60.0, 60.0);
        tree.add(30.0, 30.0, 40.0, 40.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_nearest_k(15.0, 15.0, 3, &mut results);
        assert_eq!(results.len(), 3, "K=3 should return all 3 items");
    }

    #[test]
    fn test_query_nearest_k_k_exceeds_items() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.add(50.0, 50.0, 60.0, 60.0);
        tree.add(30.0, 30.0, 40.0, 40.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_nearest_k(15.0, 15.0, 100, &mut results);
        assert_eq!(results.len(), 3, "K>items should return all items");
    }

    #[test]
    fn test_query_nearest_k_nearest_ordering() {
        let mut tree = HilbertRTree::new();
        tree.add(0.0, 0.0, 5.0, 5.0);      // 0 - closest
        tree.add(50.0, 50.0, 55.0, 55.0); // 1 - farthest
        tree.add(20.0, 20.0, 25.0, 25.0); // 2 - middle
        tree.build();

        let mut results = Vec::new();
        tree.query_nearest_k(2.0, 2.0, 3, &mut results);
        assert_eq!(results.len(), 3);

        // First result should be closest
        assert_eq!(results[0], 0, "Closest item should be first");
    }

    #[test]
    fn test_query_nearest_k_no_duplicates() {
        let mut tree = HilbertRTree::new();
        for i in 0..10 {
            tree.add(i as f64, i as f64, (i + 1) as f64, (i + 1) as f64);
        }
        tree.build();

        let mut results = Vec::new();
        tree.query_nearest_k(5.0, 5.0, 5, &mut results);

        let original_len = results.len();
        results.sort();
        results.dedup();
        assert_eq!(
            results.len(), original_len,
            "Results should have no duplicates"
        );
    }

    // ============================================================================
    // HILBERT SORTING TESTS
    // ============================================================================

    #[test]
    fn test_hilbert_sort_preserves_items() {
        let mut tree = HilbertRTree::new();
        for i in 0..20 {
            tree.add(i as f64, i as f64, (i + 1) as f64, (i + 1) as f64);
        }
        tree.build();

        assert_eq!(tree.num_items, 20, "All items should be preserved");
    }

    #[test]
    fn test_hilbert_sort_enables_queries() {
        let mut tree = HilbertRTree::new();
        let items = vec![5, 2, 8, 1, 9, 3, 7, 4, 6, 0];
        for idx in items {
            tree.add(idx as f64, idx as f64, (idx + 1) as f64, (idx + 1) as f64);
        }
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(2.0, 2.0, 8.0, 8.0, &mut results);
        assert!(!results.is_empty(), "Queries should work after Hilbert sort");
    }

    // ============================================================================
    // CONSISTENCY TESTS
    // ============================================================================

    #[test]
    fn test_repeated_queries_consistent() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.add(30.0, 30.0, 40.0, 40.0);
        tree.build();

        let mut results1 = Vec::new();
        let mut results2 = Vec::new();

        tree.query_intersecting(12.0, 12.0, 35.0, 35.0, &mut results1);
        tree.query_intersecting(12.0, 12.0, 35.0, 35.0, &mut results2);

        results1.sort();
        results2.sort();
        assert_eq!(results1, results2, "Repeated queries should give same results");
    }

    #[test]
    fn test_deterministic_results() {
        let mut tree1 = HilbertRTree::new();
        let mut tree2 = HilbertRTree::new();

        let boxes = vec![(10.0, 10.0, 20.0, 20.0), (15.0, 15.0, 25.0, 25.0)];

        for &(min_x, min_y, max_x, max_y) in &boxes {
            tree1.add(min_x, min_y, max_x, max_y);
            tree2.add(min_x, min_y, max_x, max_y);
        }

        tree1.build();
        tree2.build();

        let mut results1 = Vec::new();
        let mut results2 = Vec::new();

        tree1.query_intersecting(12.0, 12.0, 22.0, 22.0, &mut results1);
        tree2.query_intersecting(12.0, 12.0, 22.0, 22.0, &mut results2);

        results1.sort();
        results2.sort();
        assert_eq!(results1, results2, "Identical trees should give identical results");
    }

    // ============================================================================
    // EDGE CASES
    // ============================================================================

    #[test]
    fn test_very_large_coordinates() {
        let mut tree = HilbertRTree::new();
        tree.add(1e10, 1e10, 1e10 + 100.0, 1e10 + 100.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(1e10, 1e10, 1e10 + 50.0, 1e10 + 50.0, &mut results);
        assert_eq!(results.len(), 1);
    }

    #[test]
    fn test_very_small_coordinates() {
        let mut tree = HilbertRTree::new();
        tree.add(1e-10, 1e-10, 2e-10, 2e-10);
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(1e-10, 1e-10, 2e-10, 2e-10, &mut results);
        assert_eq!(results.len(), 1);
    }

    #[test]
    fn test_mixed_positive_negative() {
        let mut tree = HilbertRTree::new();
        tree.add(-10.0, -10.0, 10.0, 10.0);
        tree.add(5.0, 5.0, 15.0, 15.0);
        tree.add(-15.0, -15.0, -5.0, -5.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(-5.0, -5.0, 5.0, 5.0, &mut results);
        assert_eq!(results.len(), 3, "All 3 overlapping boxes should be found");
    }

    #[test]
    fn test_query_zero_size_box() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(15.0, 15.0, 15.0, 15.0, &mut results);
        // Behavior depends on implementation
    }

    #[test]
    fn test_query_inverted_box() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting(20.0, 20.0, 10.0, 10.0, &mut results);
        // Behavior depends on implementation
    }

    // ============================================================================
    // NEW QUERY METHOD TESTS
    // ============================================================================

    #[test]
    fn test_query_point_inside_box() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.add(30.0, 30.0, 40.0, 40.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_point(15.0, 15.0, &mut results);
        assert_eq!(results.len(), 1);
        assert_eq!(results[0], 0);
    }

    #[test]
    fn test_query_point_outside_box() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_point(5.0, 5.0, &mut results);
        assert_eq!(results.len(), 0);
    }

    #[test]
    fn test_query_point_on_boundary() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_point(10.0, 15.0, &mut results);
        assert_eq!(results.len(), 1);
    }

    #[test]
    fn test_query_contain_basic() {
        let mut tree = HilbertRTree::new();
        tree.add(5.0, 5.0, 25.0, 25.0);
        tree.add(30.0, 30.0, 40.0, 40.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_contain(10.0, 10.0, 20.0, 20.0, &mut results);
        results.sort();
        assert_eq!(results, vec![0]);
    }

    #[test]
    fn test_query_contain_multiple() {
        let mut tree = HilbertRTree::new();
        tree.add(0.0, 0.0, 50.0, 50.0);
        tree.add(10.0, 10.0, 40.0, 40.0);
        tree.add(30.0, 30.0, 40.0, 40.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_contain(20.0, 20.0, 30.0, 30.0, &mut results);
        results.sort();
        assert_eq!(results.len(), 2);
    }

    #[test]
    fn test_query_contained_within_basic() {
        let mut tree = HilbertRTree::new();
        tree.add(15.0, 15.0, 25.0, 25.0);
        tree.add(30.0, 30.0, 40.0, 40.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_contained_within(10.0, 10.0, 50.0, 50.0, &mut results);
        results.sort();
        assert_eq!(results, vec![0, 1]);
    }

    #[test]
    fn test_query_contained_within_partial() {
        let mut tree = HilbertRTree::new();
        tree.add(5.0, 5.0, 25.0, 25.0);
        tree.add(30.0, 30.0, 40.0, 40.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_contained_within(10.0, 10.0, 50.0, 50.0, &mut results);
        results.sort();
        assert_eq!(results, vec![1]);  // Only box 1 (30,30)-(40,40) is contained in (10,10)-(50,50)
    }

    #[test]
    fn test_query_nearest_basic() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.add(50.0, 50.0, 60.0, 60.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_nearest_k(15.0, 15.0, 1, &mut results);
        assert_eq!(results, vec![0]);
    }

    #[test]
    fn test_query_nearest_empty_tree() {
        let tree = HilbertRTree::new();
        let mut results = Vec::new();
        tree.query_nearest_k(15.0, 15.0, 1, &mut results);
        assert_eq!(results, vec![]);
    }

    #[test]
    fn test_query_intersecting_k_basic() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.add(15.0, 15.0, 25.0, 25.0);
        tree.add(30.0, 30.0, 40.0, 40.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting_k(12.0, 12.0, 22.0, 22.0, 2, &mut results);
        assert_eq!(results.len(), 2);
    }

    #[test]
    fn test_query_intersecting_k_zero() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_intersecting_k(15.0, 15.0, 25.0, 25.0, 0, &mut results);
        assert_eq!(results.len(), 0);
    }

    #[test]
    fn test_query_circle_basic() {
        let mut tree = HilbertRTree::new();
        tree.add(8.0, 8.0, 12.0, 12.0);
        tree.add(50.0, 50.0, 52.0, 52.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_circle(10.0, 10.0, 5.0, &mut results);
        results.sort();
        assert_eq!(results, vec![0]);
    }

    #[test]
    fn test_query_circle_empty() {
        let mut tree = HilbertRTree::new();
        tree.add(50.0, 50.0, 52.0, 52.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_circle(10.0, 10.0, 1.0, &mut results);
        assert_eq!(results.len(), 0);
    }

    #[test]
    fn test_query_in_direction_basic() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.add(50.0, 50.0, 60.0, 60.0);
        tree.build();

        let mut results = Vec::new();
        // Starting rectangle (15,15)-(25,25), moving down-right (1,1) for distance 30
        // Normalized direction: (1/√2, 1/√2) ≈ (0.707, 0.707)
        // Movement: (0.707*30, 0.707*30) ≈ (21.2, 21.2)
        // Sweep area: (15, 15)-(46.2, 46.2), captures box 0 only
        tree.query_in_direction(15.0, 15.0, 25.0, 25.0, 1.0, 1.0, 30.0, &mut results);
        results.sort();
        assert_eq!(results.len(), 1);
    }

    #[test]
    fn test_query_in_direction_none() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_in_direction(50.0, 50.0, 60.0, 60.0, 1.0, 0.0, 10.0, &mut results);
        assert_eq!(results.len(), 0);
    }

    #[test]
    fn test_query_in_direction_k_basic() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 15.0, 15.0);
        tree.add(30.0, 30.0, 35.0, 35.0);
        tree.add(60.0, 60.0, 65.0, 65.0);
        tree.build();

        let mut results = Vec::new();
        // Move diagonally to capture multiple boxes
        tree.query_in_direction_k(15.0, 15.0, 25.0, 25.0, 1.0, 1.0, 2, 50.0, &mut results);
        assert_eq!(results.len(), 2);
    }

    #[test]
    fn test_query_in_direction_k_zero() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_in_direction_k(15.0, 15.0, 25.0, 25.0, 1.0, 0.0, 0, 30.0, &mut results);
        assert_eq!(results.len(), 0);
    }

    // ============================================================================
    // QUERY_INTERSECTING_ID TESTS
    // ============================================================================

    #[test]
    fn test_query_intersecting_id_basic() {
        let mut tree = HilbertRTree::new();
        tree.add(0.0, 0.0, 2.0, 2.0);      // Box 0
        tree.add(1.0, 1.0, 3.0, 3.0);      // Box 1 - overlaps with 0
        tree.add(1.5, 1.5, 2.5, 2.5);      // Box 2 - overlaps with 0 and 1
        tree.add(5.0, 5.0, 6.0, 6.0);      // Box 3 - no overlap
        tree.build();

        let mut results = Vec::new();
        let res = tree.query_intersecting_id(0, &mut results);
        if res.is_err() {
            panic!("query_intersecting_id should succeed");
        }
        results.sort();

        // Box 0 should intersect with boxes 1 and 2, but not itself
        if results != vec![1, 2] {
            panic!("Box 0 query failed: expected [1, 2], got {:?}", results);
        }
    }

    #[test]
    fn test_query_intersecting_id_no_overlaps() {
        let mut tree = HilbertRTree::new();
        tree.add(0.0, 0.0, 1.0, 1.0);      // Box 0
        tree.add(5.0, 5.0, 6.0, 6.0);      // Box 1 - no overlap
        tree.build();

        let mut results = Vec::new();
        let res = tree.query_intersecting_id(0, &mut results);
        if res.is_err() {
            panic!("query_intersecting_id should succeed");
        }

        if !results.is_empty() {
            panic!("Box 0 query should return empty, got {:?}", results);
        }
    }

    #[test]
    fn test_query_intersecting_id_excludes_self() {
        let mut tree = HilbertRTree::new();
        tree.add(0.0, 0.0, 1.0, 1.0);
        tree.build();

        let mut results = Vec::new();
        let res = tree.query_intersecting_id(0, &mut results);
        if res.is_err() {
            panic!("query_intersecting_id should succeed");
        }

        // Should never include the query item itself
        if results.contains(&0) {
            panic!("Query should not include self, got {:?}", results);
        }
    }

    #[test]
    fn test_query_intersecting_id_bounds_check() {
        let mut tree = HilbertRTree::new();
        tree.add(0.0, 0.0, 1.0, 1.0);
        tree.build();

        let mut results = Vec::new();
        let res = tree.query_intersecting_id(5, &mut results); // Out of bounds
        if res.is_ok() {
            panic!("query_intersecting_id should fail with out of bounds error");
        }
        if !res.unwrap_err().contains("out of bounds") {
            panic!("Error message should mention 'out of bounds'");
        }
    }

    #[test]
    fn test_query_intersecting_id_multiple_overlaps() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);  // Box 0
        tree.add(15.0, 15.0, 25.0, 25.0);  // Box 1 - overlaps with 0
        tree.add(12.0, 12.0, 18.0, 18.0);  // Box 2 - overlaps with 0 and 1
        tree.add(11.0, 11.0, 19.0, 19.0);  // Box 3 - overlaps with 0, 1, 2
        tree.add(50.0, 50.0, 60.0, 60.0);  // Box 4 - no overlap
        tree.build();

        let mut results = Vec::new();
        let res = tree.query_intersecting_id(0, &mut results);
        if res.is_err() {
            panic!("query_intersecting_id should succeed");
        }
        results.sort();

        // Box 0 should intersect with boxes 1, 2, 3
        if results != vec![1, 2, 3] {
            panic!("Box 0 query failed: expected [1, 2, 3], got {:?}", results);
        }
    }

    #[test]
    fn test_query_intersecting_id_different_items() {
        let mut tree = HilbertRTree::new();
        tree.add(0.0, 0.0, 2.0, 2.0);      // Box 0
        tree.add(1.0, 1.0, 3.0, 3.0);      // Box 1 - overlaps with 0
        tree.add(1.5, 1.5, 2.5, 2.5);      // Box 2 - overlaps with 0 and 1
        tree.add(5.0, 5.0, 6.0, 6.0);      // Box 3 - no overlap
        tree.build();

        // Query with box 1
        let mut results = Vec::new();
        let res = tree.query_intersecting_id(1, &mut results);
        if res.is_err() {
            panic!("query_intersecting_id should succeed");
        }
        results.sort();

        // Box 1 should intersect with boxes 0 and 2, but not itself
        if results != vec![0, 2] {
            panic!("Box 1 query failed: expected [0, 2], got {:?}", results);
        }
    }

    #[test]
    fn test_query_intersecting_id_isolated_item() {
        let mut tree = HilbertRTree::new();
        tree.add(0.0, 0.0, 1.0, 1.0);      // Box 0
        tree.add(2.0, 2.0, 3.0, 3.0);      // Box 1
        tree.add(5.0, 5.0, 6.0, 6.0);      // Box 2
        tree.build();

        let mut results = Vec::new();
        let res = tree.query_intersecting_id(2, &mut results);
        if res.is_err() {
            panic!("query_intersecting_id should succeed");
        }

        if !results.is_empty() {
            panic!("Isolated box query should return empty, got {:?}", results);
        }
    }

    // ============================================================================
    // GET (id -> box lookup) TESTS
    // ============================================================================

    /// Deterministic pseudo-random boxes (no external RNG needed in unit tests).
    fn scattered_boxes(n: usize) -> Vec<(f64, f64, f64, f64)> {
        let mut state = 12345_u64;
        let mut next = || {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            (state >> 33) as f64 / (1_u64 << 31) as f64 * 100.0
        };
        (0..n)
            .map(|_| {
                let (x, y) = (next(), next());
                (x, y, x + 1.0, y + 1.0)
            })
            .collect()
    }

    #[test]
    fn test_get_returns_box_for_every_id_after_sorting_build() {
        // More than node_size items, so build() reorders the boxes.
        let boxes = scattered_boxes(1000);
        let mut tree = HilbertRTree::new();
        for b in &boxes {
            tree.add(b.0, b.1, b.2, b.3);
        }
        tree.build();

        for (id, b) in boxes.iter().enumerate() {
            assert_eq!(tree.get(id), Some(*b), "get({}) returned the wrong box", id);
        }
        assert_eq!(tree.get(boxes.len()), None, "Out-of-range id should return None");
    }

    #[test]
    fn test_get_on_cloned_tree() {
        let boxes = scattered_boxes(500);
        let mut tree = HilbertRTree::new();
        for b in &boxes {
            tree.add(b.0, b.1, b.2, b.3);
        }
        tree.build();
        assert_eq!(tree.get(0), Some(boxes[0])); // fills the lookup table before cloning

        let copy = tree.clone();
        for (id, b) in boxes.iter().enumerate() {
            assert_eq!(copy.get(id), Some(*b), "clone: get({}) returned the wrong box", id);
        }
    }

    #[test]
    fn test_get_after_save_and_load() {
        let boxes = scattered_boxes(500);
        let mut tree = HilbertRTree::new();
        for b in &boxes {
            tree.add(b.0, b.1, b.2, b.3);
        }
        tree.build();
        assert_eq!(tree.get(0), Some(boxes[0]));

        let path = std::env::temp_dir().join("aabb_get_after_load_test.bin");
        tree.save(&path).expect("save should succeed");
        let loaded = HilbertRTree::load(&path).expect("load should succeed");
        std::fs::remove_file(&path).expect("temp file should be removable");

        for (id, b) in boxes.iter().enumerate() {
            assert_eq!(loaded.get(id), Some(*b), "loaded: get({}) returned the wrong box", id);
        }
    }

    // ============================================================================
    // REBUILD / CLEAR TESTS: ids must stay the ones given by add()
    // ============================================================================

    type Rect = (f64, f64, f64, f64);

    fn add_all(tree: &mut HilbertRTree, boxes: &[Rect]) {
        for b in boxes {
            tree.add(b.0, b.1, b.2, b.3);
        }
    }

    /// Every id returns its own box, and a query returns exactly the ids a brute-force scan finds.
    fn assert_tree_matches(tree: &HilbertRTree, boxes: &[Rect], context: &str) {
        assert_eq!(tree.len(), boxes.len(), "{}: wrong item count", context);
        for (id, b) in boxes.iter().enumerate() {
            assert_eq!(tree.get(id), Some(*b), "{}: get({}) returned the wrong box", context, id);
        }
        for q in [(20.0, 20.0, 40.0, 40.0), (0.0, 0.0, 100.0, 100.0), (70.0, 10.0, 71.0, 12.0)] {
            let mut got = Vec::new();
            tree.query_intersecting(q.0, q.1, q.2, q.3, &mut got);
            got.sort_unstable();
            let want: Vec<usize> = boxes
                .iter()
                .enumerate()
                .filter(|(_, b)| b.0 <= q.2 && b.2 >= q.0 && b.1 <= q.3 && b.3 >= q.1)
                .map(|(i, _)| i)
                .collect();
            assert_eq!(got, want, "{}: query {:?} returned the wrong ids", context, q);
        }
    }

    #[test]
    fn test_build_twice_keeps_ids() {
        let boxes = scattered_boxes(10_000);
        let mut tree = HilbertRTree::new();
        add_all(&mut tree, &boxes);
        tree.build();
        tree.build();
        assert_tree_matches(&tree, &boxes, "second build");
        tree.build();
        assert_tree_matches(&tree, &boxes, "third build");
    }

    #[test]
    fn test_add_after_build_then_rebuild_keeps_ids() {
        let all = scattered_boxes(3_000);
        let mut tree = HilbertRTree::new();
        let mut added = 0;
        // Frames of different sizes: add, build, check, repeat.
        for frame_size in [1000, 1, 500, 700, 799] {
            add_all(&mut tree, &all[added..added + frame_size]);
            added += frame_size;
            tree.build();
            assert_tree_matches(&tree, &all[..added], &format!("after {} items", added));
        }
    }

    #[test]
    fn test_rebuild_small_tree_growing_past_node_size() {
        // Up to node_size (16) items the boxes are not reordered; past it they are.
        let all = scattered_boxes(60);
        let mut tree = HilbertRTree::new();
        let mut added = 0;
        for frame_size in [5, 5, 5, 1, 4, 20, 20] {
            add_all(&mut tree, &all[added..added + frame_size]);
            added += frame_size;
            tree.build();
            assert_tree_matches(&tree, &all[..added], &format!("after {} items", added));
        }
    }

    #[test]
    fn test_get_between_add_and_build() {
        let all = scattered_boxes(400);
        let mut tree = HilbertRTree::new();
        add_all(&mut tree, &all[..300]);
        tree.build();
        add_all(&mut tree, &all[300..]);

        // Not built yet: get() works for old and new items, queries return nothing.
        for (id, b) in all.iter().enumerate() {
            assert_eq!(tree.get(id), Some(*b), "get({}) before rebuild", id);
        }
        let mut results = vec![99];
        tree.query_intersecting(0.0, 0.0, 100.0, 100.0, &mut results);
        assert!(results.is_empty(), "queries should return nothing until build() is called");

        tree.build();
        assert_tree_matches(&tree, &all, "after rebuild");
    }

    #[test]
    fn test_clear_and_reuse() {
        let first = scattered_boxes(2_000);
        let mut tree = HilbertRTree::with_capacity(first.len());
        add_all(&mut tree, &first);
        tree.build();
        assert_tree_matches(&tree, &first, "first fill");

        for (frame, count) in [(0, 2_000), (1, 500), (2, 3_000), (3, 10)] {
            tree.clear();
            assert!(tree.is_empty(), "frame {}: tree should be empty after clear()", frame);
            assert_eq!(tree.get(0), None, "frame {}: no items after clear()", frame);
            let boxes: Vec<Rect> = scattered_boxes(count + 7 * frame)
                .into_iter()
                .map(|b| (b.0 * 0.5, b.1 * 0.5, b.2 * 0.5, b.3 * 0.5))
                .collect();
            add_all(&mut tree, &boxes);
            tree.build();
            assert_tree_matches(&tree, &boxes, &format!("refill {}", frame));
        }
    }

    #[test]
    fn test_clear_on_empty_and_unbuilt_tree() {
        let mut tree = HilbertRTree::new();
        tree.clear();
        tree.build();
        assert!(tree.is_empty());

        tree.add(0.0, 0.0, 1.0, 1.0);
        tree.clear(); // cleared before it was ever built
        tree.add(5.0, 5.0, 6.0, 6.0);
        tree.build();
        assert_eq!(tree.get(0), Some((5.0, 5.0, 6.0, 6.0)));
        assert_eq!(tree.len(), 1);
    }

    #[test]
    fn test_rebuild_and_clear_keep_bounds_correct() {
        // After clear() the bounds must restart; stale bounds would break the Hilbert order.
        let mut tree = HilbertRTree::new();
        tree.add(0.0, 0.0, 1000.0, 1000.0);
        tree.build();
        tree.clear();
        let boxes = scattered_boxes(300);
        add_all(&mut tree, &boxes);
        tree.build();
        assert_tree_matches(&tree, &boxes, "after clear with new bounds");
    }

    #[test]
    fn test_save_after_rebuild_and_add_after_load() {
        let all = scattered_boxes(1_200);
        let mut tree = HilbertRTree::new();
        add_all(&mut tree, &all[..1_000]);
        tree.build();
        add_all(&mut tree, &all[1_000..1_100]);
        tree.build();

        let path = std::env::temp_dir().join("aabb_rebuild_save_load_test.bin");
        tree.save(&path).expect("save should succeed");
        let mut loaded = HilbertRTree::load(&path).expect("load should succeed");
        std::fs::remove_file(&path).expect("temp file should be removable");
        assert_tree_matches(&loaded, &all[..1_100], "loaded");

        // A loaded tree can also take more items and be rebuilt.
        add_all(&mut loaded, &all[1_100..]);
        loaded.build();
        assert_tree_matches(&loaded, &all, "loaded then extended");
    }

    #[test]
    fn test_save_refuses_tree_with_unbuilt_changes() {
        let all = scattered_boxes(100);
        let mut tree = HilbertRTree::new();
        add_all(&mut tree, &all[..50]);
        tree.build();
        tree.add(1.0, 1.0, 2.0, 2.0);

        let path = std::env::temp_dir().join("aabb_unbuilt_save_test.bin");
        drop(std::fs::remove_file(&path)); // ignore: file normally does not exist
        assert!(tree.save(&path).is_err(), "save() must not write a tree with unbuilt changes");
        assert!(!path.exists(), "no file should be written");
    }

    #[test]
    fn test_save_after_clear_does_not_write_stale_buffer() {
        let mut tree = HilbertRTree::new();
        add_all(&mut tree, &scattered_boxes(5_000));
        tree.build();
        tree.clear();
        let small = scattered_boxes(100);
        add_all(&mut tree, &small);
        tree.build();

        let path = std::env::temp_dir().join("aabb_clear_save_test.bin");
        tree.save(&path).expect("save should succeed");
        let size = std::fs::metadata(&path).expect("file should exist").len();
        std::fs::remove_file(&path).expect("temp file should be removable");
        assert!(size < 20_000, "file should hold 100 items, not the old 5000 ({} bytes)", size);
    }

    #[test]
    fn test_query_intersecting_id_uses_ids_not_leaf_positions() {
        let boxes = scattered_boxes(2_000);
        let mut tree = HilbertRTree::new();
        add_all(&mut tree, &boxes);
        tree.build();
        // Rebuild once more with extra items so both build paths are covered.
        let extra = scattered_boxes(2_100);
        add_all(&mut tree, &extra[2_000..]);
        tree.build();
        let all: Vec<Rect> = boxes.iter().chain(extra[2_000..].iter()).copied().collect();

        for id in (0..all.len()).step_by(37) {
            let mut got = Vec::new();
            tree.query_intersecting_id(id, &mut got).expect("id is in range");
            got.sort_unstable();
            let q = all[id];
            let want: Vec<usize> = all
                .iter()
                .enumerate()
                .filter(|(i, b)| *i != id && b.0 <= q.2 && b.2 >= q.0 && b.1 <= q.3 && b.3 >= q.1)
                .map(|(i, _)| i)
                .collect();
            assert_eq!(got, want, "query_intersecting_id({}) returned the wrong ids", id);
        }
    }
}

