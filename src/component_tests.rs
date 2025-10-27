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
        assert!(!tree.level_bounds.is_empty(), "Should have level bounds");
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
    fn test_query_containing_basic() {
        let mut tree = HilbertRTree::new();
        tree.add(5.0, 5.0, 25.0, 25.0);
        tree.add(30.0, 30.0, 40.0, 40.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_containing(10.0, 10.0, 20.0, 20.0, &mut results);
        results.sort();
        assert_eq!(results, vec![0]);
    }

    #[test]
    fn test_query_containing_multiple() {
        let mut tree = HilbertRTree::new();
        tree.add(0.0, 0.0, 50.0, 50.0);
        tree.add(10.0, 10.0, 40.0, 40.0);
        tree.add(30.0, 30.0, 40.0, 40.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_containing(20.0, 20.0, 30.0, 30.0, &mut results);
        results.sort();
        assert_eq!(results.len(), 2);
    }

    #[test]
    fn test_query_contained_by_basic() {
        let mut tree = HilbertRTree::new();
        tree.add(15.0, 15.0, 25.0, 25.0);
        tree.add(30.0, 30.0, 40.0, 40.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_contained_by(10.0, 10.0, 50.0, 50.0, &mut results);
        results.sort();
        assert_eq!(results, vec![0, 1]);
    }

    #[test]
    fn test_query_contained_by_partial() {
        let mut tree = HilbertRTree::new();
        tree.add(5.0, 5.0, 25.0, 25.0);
        tree.add(30.0, 30.0, 40.0, 40.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_contained_by(10.0, 10.0, 50.0, 50.0, &mut results);
        results.sort();
        assert_eq!(results, vec![1]);  // Only box 1 (30,30)-(40,40) is contained in (10,10)-(50,50)
    }

    #[test]
    fn test_query_nearest_basic() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 20.0, 20.0);
        tree.add(50.0, 50.0, 60.0, 60.0);
        tree.build();

        let result = tree.query_nearest(15.0, 15.0);
        assert_eq!(result, Some(0));
    }

    #[test]
    fn test_query_nearest_empty_tree() {
        let tree = HilbertRTree::new();
        let result = tree.query_nearest(15.0, 15.0);
        assert_eq!(result, None);
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
    fn test_query_within_distance_basic() {
        let mut tree = HilbertRTree::new();
        tree.add(10.0, 10.0, 12.0, 12.0);
        tree.add(50.0, 50.0, 52.0, 52.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_within_distance(11.0, 11.0, 10.0, &mut results);
        results.sort();
        assert_eq!(results.len(), 1);
        assert_eq!(results[0], 0);
    }

    #[test]
    fn test_query_within_distance_multiple() {
        let mut tree = HilbertRTree::new();
        tree.add(0.0, 0.0, 2.0, 2.0);
        tree.add(3.0, 3.0, 5.0, 5.0);
        tree.add(50.0, 50.0, 52.0, 52.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_within_distance(1.0, 1.0, 10.0, &mut results);
        results.sort();
        assert!(results.len() >= 2);
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
}
