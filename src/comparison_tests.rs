//! Comparison tests between HilbertRTree (hierarchical) and HilbertRTreeLeg (flat sorted)

#[cfg(test)]
mod tests {
    use crate::{HilbertRTree, HilbertRTreeLeg};
    use rand::{Rng, SeedableRng};

    /// Helper to add same boxes to both trees
    fn setup_trees(boxes: &[(f64, f64, f64, f64)]) -> (HilbertRTree, HilbertRTreeLeg) {
        let mut tree_new = HilbertRTree::with_capacity(boxes.len());
        let mut tree_leg = HilbertRTreeLeg::new();

        for &(min_x, min_y, max_x, max_y) in boxes {
            tree_new.add(min_x, min_y, max_x, max_y);
            tree_leg.add(min_x, min_y, max_x, max_y);
        }

        tree_new.build();
        tree_leg.build();

        (tree_new, tree_leg)
    }

    // NOTE: HilbertRTree hierarchical implementation has index tracking issues
    // Temporarily disabled while debugging
    #[test]
    fn test_basic_query_consistency() {
        let boxes = vec![
            (10.0, 10.0, 20.0, 20.0),
            (30.0, 30.0, 40.0, 40.0),
            (15.0, 15.0, 35.0, 35.0),
            (5.0, 5.0, 12.0, 12.0),
        ];

        let (tree_new, tree_leg) = setup_trees(&boxes);

        let mut results_new = Vec::new();
        let mut results_leg = Vec::new();

        tree_new.query_intersecting(12.0, 12.0, 18.0, 18.0, &mut results_new);
        tree_leg.query_intersecting(12.0, 12.0, 18.0, 18.0, &mut results_leg);

        // Both should find results
        assert!(!results_new.is_empty(), "HilbertRTree found no results");
        assert!(!results_leg.is_empty(), "HilbertRTreeLeg found no results");

        // Results should have same items (may be in different order)
        results_new.sort();
        results_leg.sort();
        assert_eq!(
            results_new, results_leg,
            "Query results differ between implementations"
        );
    }

    #[test]
    fn test_empty_query_consistency() {
        let boxes = vec![
            (10.0, 10.0, 20.0, 20.0),
            (30.0, 30.0, 40.0, 40.0),
            (50.0, 50.0, 60.0, 60.0),
        ];

        let (tree_new, tree_leg) = setup_trees(&boxes);

        let mut results_new = Vec::new();
        let mut results_leg = Vec::new();

        // Query area that should return no results
        tree_new.query_intersecting(70.0, 70.0, 80.0, 80.0, &mut results_new);
        tree_leg.query_intersecting(70.0, 70.0, 80.0, 80.0, &mut results_leg);

        assert_eq!(results_new.len(), 0, "HilbertRTree returned unexpected results");
        assert_eq!(results_leg.len(), 0, "HilbertRTreeLeg returned unexpected results");
    }

    #[test]
    fn test_large_dataset_consistency() {
        let mut rng = rand::rngs::StdRng::seed_from_u64(42);
        let mut boxes = Vec::new();

        for _ in 0..1000 {
            let min_x = rng.random_range(0.0..100.0);
            let min_y = rng.random_range(0.0..100.0);
            let max_x = min_x + rng.random_range(0.1..5.0);
            let max_y = min_y + rng.random_range(0.1..5.0);

            boxes.push((min_x, min_y, max_x, max_y));
        }

        let (tree_new, tree_leg) = setup_trees(&boxes);

        let mut results_new = Vec::new();
        let mut results_leg = Vec::new();

        tree_new.query_intersecting(50.0, 50.0, 60.0, 60.0, &mut results_new);
        tree_leg.query_intersecting(50.0, 50.0, 60.0, 60.0, &mut results_leg);

        // Results should be the same
        results_new.sort();
        results_leg.sort();
        assert_eq!(
            results_new, results_leg,
            "Large dataset query results differ: new={:?}, leg={:?}",
            results_new, results_leg
        );
    }

    #[test]
    fn test_nearest_neighbor_consistency() {
        let boxes = vec![
            (10.0, 10.0, 20.0, 20.0),
            (30.0, 30.0, 40.0, 40.0),
            (50.0, 50.0, 60.0, 60.0),
            (25.0, 25.0, 28.0, 28.0),
        ];

        let (tree_new, tree_leg) = setup_trees(&boxes);

        let mut results_new = Vec::new();
        let mut results_leg = Vec::new();

        tree_new.query_nearest_k(15.0, 15.0, 2, &mut results_new);
        tree_leg.query_nearest_k(15.0, 15.0, 2, &mut results_leg);

        // Both should return K results
        assert_eq!(
            results_new.len(),
            2,
            "HilbertRTree should return 2 neighbors"
        );
        assert_eq!(
            results_leg.len(),
            2,
            "HilbertRTreeLeg should return 2 neighbors"
        );

        results_new.sort();
        results_leg.sort();
        assert_eq!(
            results_new, results_leg,
            "Nearest neighbor results differ"
        );
    }

    #[test]
    fn test_overlapping_boxes() {
        let boxes = vec![
            (0.0, 0.0, 10.0, 10.0),
            (5.0, 5.0, 15.0, 15.0),
            (2.0, 2.0, 8.0, 8.0),
            (7.0, 7.0, 12.0, 12.0),
        ];

        let (tree_new, tree_leg) = setup_trees(&boxes);

        let mut results_new = Vec::new();
        let mut results_leg = Vec::new();

        tree_new.query_intersecting(6.0, 6.0, 9.0, 9.0, &mut results_new);
        tree_leg.query_intersecting(6.0, 6.0, 9.0, 9.0, &mut results_leg);

        results_new.sort();
        results_leg.sort();
        assert_eq!(
            results_new, results_leg,
            "Overlapping box query results differ"
        );
    }

    #[test]
    fn test_query_point_consistency() {
        let boxes = vec![
            (10.0, 10.0, 20.0, 20.0),
            (30.0, 30.0, 40.0, 40.0),
            (15.0, 15.0, 35.0, 35.0),
        ];

        let (tree_new, tree_leg) = setup_trees(&boxes);

        let mut results_new = Vec::new();
        let mut results_leg = Vec::new();

        tree_new.query_point(25.0, 25.0, &mut results_new);
        tree_leg.query_point(25.0, 25.0, &mut results_leg);

        results_new.sort();
        results_leg.sort();
        assert_eq!(results_new, results_leg, "Point query results differ");
    }

    #[test]
    fn test_query_contain_consistency() {
        let boxes = vec![
            (5.0, 5.0, 50.0, 50.0),
            (10.0, 10.0, 20.0, 20.0),
            (30.0, 30.0, 40.0, 40.0),
        ];

        let (tree_new, tree_leg) = setup_trees(&boxes);

        let mut results_new = Vec::new();
        let mut results_leg = Vec::new();

        tree_new.query_contain(15.0, 15.0, 25.0, 25.0, &mut results_new);
        tree_leg.query_contain(15.0, 15.0, 25.0, 25.0, &mut results_leg);

        results_new.sort();
        results_leg.sort();
        assert_eq!(
            results_new, results_leg,
            "query_contain results differ"
        );
    }

    #[test]
    fn test_query_contained_within_consistency() {
        let boxes = vec![
            (10.0, 10.0, 20.0, 20.0),
            (30.0, 30.0, 40.0, 40.0),
            (15.0, 15.0, 25.0, 25.0),
        ];

        let (tree_new, tree_leg) = setup_trees(&boxes);

        let mut results_new = Vec::new();
        let mut results_leg = Vec::new();

        tree_new.query_contained_within(5.0, 5.0, 50.0, 50.0, &mut results_new);
        tree_leg.query_contained_within(5.0, 5.0, 50.0, 50.0, &mut results_leg);

        results_new.sort();
        results_leg.sort();
        assert_eq!(
            results_new, results_leg,
            "query_contained_within results differ"
        );
    }

    #[test]
    fn test_query_nearest_consistency() {
        let boxes = vec![
            (10.0, 10.0, 20.0, 20.0),
            (50.0, 50.0, 60.0, 60.0),
            (25.0, 25.0, 35.0, 35.0),
        ];

        let (tree_new, tree_leg) = setup_trees(&boxes);

        let result_new = tree_new.query_nearest(15.0, 15.0);
        let result_leg = tree_leg.query_nearest(15.0, 15.0);

        assert_eq!(result_new, result_leg, "query_nearest results differ");
    }

    #[test]
    fn test_query_intersecting_k_consistency() {
        let boxes = vec![
            (10.0, 10.0, 20.0, 20.0),
            (15.0, 15.0, 25.0, 25.0),
            (30.0, 30.0, 40.0, 40.0),
            (35.0, 35.0, 45.0, 45.0),
        ];

        let (tree_new, tree_leg) = setup_trees(&boxes);

        let mut results_new = Vec::new();
        let mut results_leg = Vec::new();

        tree_new.query_intersecting_k(12.0, 12.0, 22.0, 22.0, 2, &mut results_new);
        tree_leg.query_intersecting_k(12.0, 12.0, 22.0, 22.0, 2, &mut results_leg);

        results_new.sort();
        results_leg.sort();
        assert_eq!(
            results_new, results_leg,
            "query_intersecting_k results differ"
        );
    }

    #[test]
    fn test_query_circle_consistency() {
        let boxes = vec![
            (8.0, 8.0, 12.0, 12.0),
            (50.0, 50.0, 52.0, 52.0),
            (20.0, 20.0, 25.0, 25.0),
        ];

        let (tree_new, tree_leg) = setup_trees(&boxes);

        let mut results_new = Vec::new();
        let mut results_leg = Vec::new();

        tree_new.query_circle(10.0, 10.0, 15.0, &mut results_new);
        tree_leg.query_circle(10.0, 10.0, 15.0, &mut results_leg);

        results_new.sort();
        results_leg.sort();
        assert_eq!(results_new, results_leg, "query_circle results differ");
    }

    #[test]
    fn test_query_in_direction_consistency() {
        let boxes = vec![
            (10.0, 10.0, 15.0, 15.0),
            (30.0, 30.0, 35.0, 35.0),
            (50.0, 50.0, 55.0, 55.0),
        ];

        let (tree_new, tree_leg) = setup_trees(&boxes);

        let mut results_new = Vec::new();
        let mut results_leg = Vec::new();

        tree_new.query_in_direction(15.0, 15.0, 25.0, 25.0, 1.0, 1.0, 50.0, &mut results_new);
        tree_leg.query_in_direction(15.0, 15.0, 25.0, 25.0, 1.0, 1.0, 50.0, &mut results_leg);

        results_new.sort();
        results_leg.sort();
        assert_eq!(
            results_new, results_leg,
            "query_in_direction results differ"
        );
    }

    #[test]
    fn test_query_in_direction_k_consistency() {
        let boxes = vec![
            (10.0, 10.0, 15.0, 15.0),
            (30.0, 30.0, 35.0, 35.0),
            (50.0, 50.0, 55.0, 55.0),
            (70.0, 70.0, 75.0, 75.0),
        ];

        let (tree_new, tree_leg) = setup_trees(&boxes);

        let mut results_new = Vec::new();
        let mut results_leg = Vec::new();

        tree_new.query_in_direction_k(15.0, 15.0, 25.0, 25.0, 1.0, 1.0, 2, 80.0, &mut results_new);
        tree_leg.query_in_direction_k(15.0, 15.0, 25.0, 25.0, 1.0, 1.0, 2, 80.0, &mut results_leg);

        results_new.sort();
        results_leg.sort();
        assert_eq!(
            results_new, results_leg,
            "query_in_direction_k results differ"
        );
    }
}
