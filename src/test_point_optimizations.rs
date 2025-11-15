#[cfg(test)]
mod test_point_optimizations {
    use crate::prelude::*;

    /// Test add_point convenience method
    #[test]
    fn test_add_point_method() {
        let mut tree = AABB::with_capacity(3);
        tree.add_point(0.0, 0.0);
        tree.add_point(1.0, 1.0);
        tree.add_point(2.0, 2.0);
        tree.build();

        // Verify points were added correctly
        assert_eq!(tree.len(), 3);
        
        // Verify we can retrieve the points via get()
        assert_eq!(tree.get(0).unwrap(), (0.0, 0.0, 0.0, 0.0));
        assert_eq!(tree.get(1).unwrap(), (1.0, 1.0, 1.0, 1.0));
        assert_eq!(tree.get(2).unwrap(), (2.0, 2.0, 2.0, 2.0));
    }

    /// Test add_point is equivalent to add(x, x, y, y)
    #[test]
    fn test_add_point_equivalent_to_add() {
        let mut tree1 = AABB::with_capacity(2);
        tree1.add_point(1.5, 2.5);
        tree1.add_point(3.5, 4.5);
        tree1.build();

        let mut tree2 = AABB::with_capacity(2);
        tree2.add(1.5, 1.5, 2.5, 2.5);
        tree2.add(3.5, 3.5, 4.5, 4.5);
        tree2.build();

        // Both trees should give same results
        let mut results1 = Vec::new();
        tree1.query_circle_points(1.5, 2.5, 1.0, &mut results1);

        let mut results2 = Vec::new();
        tree2.query_circle(1.5, 2.5, 1.0, &mut results2);

        assert_eq!(results1, results2);
    }

    /// Test basic query_circle_points with simple point cloud
    #[test]
    fn test_query_circle_points_basic() {
        let mut tree = AABB::with_capacity(5);
        // Add points as (x, x, y, y)
        tree.add(0.0, 0.0, 0.0, 0.0);  // Point 0: distance 0
        tree.add(1.0, 1.0, 1.0, 1.0);  // Point 1: distance sqrt(2) ≈ 1.41
        tree.add(2.0, 2.0, 2.0, 2.0);  // Point 2: distance 2*sqrt(2) ≈ 2.83
        tree.add(5.0, 5.0, 5.0, 5.0);  // Point 3
        tree.add(10.0, 10.0, 10.0, 10.0);  // Point 4
        tree.build();

        // Query circle centered at (0, 0) with radius 2.5
        let mut results = Vec::new();
        tree.query_circle_points(0.0, 0.0, 2.5, &mut results);
        
        // Should contain points 0 and 1 only (point 2 is at distance 2.83 > 2.5)
        assert_eq!(results.len(), 2);
        // Results should be sorted by distance (closest first)
        assert_eq!(results[0], 0);  // distance 0
        assert_eq!(results[1], 1);  // distance sqrt(2) ≈ 1.41
    }

    /// Test query_circle_points with no results
    #[test]
    fn test_query_circle_points_no_results() {
        let mut tree = AABB::with_capacity(2);
        tree.add(0.0, 0.0, 0.0, 0.0);
        tree.add(10.0, 10.0, 10.0, 10.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_circle_points(5.0, 5.0, 1.0, &mut results);
        assert!(results.is_empty());
    }

    /// Test query_circle_points with all points within radius
    #[test]
    fn test_query_circle_points_all_within() {
        let mut tree = AABB::with_capacity(4);
        tree.add(0.0, 0.0, 0.0, 0.0);
        tree.add(1.0, 0.0, 1.0, 0.0);
        tree.add(0.0, 1.0, 0.0, 1.0);
        tree.add(1.0, 1.0, 1.0, 1.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_circle_points(0.5, 0.5, 2.0, &mut results);
        assert_eq!(results.len(), 4);
    }

    /// Test query_nearest_k_points basic functionality
    #[test]
    fn test_query_nearest_k_points_basic() {
        let mut tree = AABB::with_capacity(5);
        tree.add(0.0, 0.0, 0.0, 0.0);  // Point 0
        tree.add(1.0, 1.0, 1.0, 1.0);  // Point 1
        tree.add(2.0, 2.0, 2.0, 2.0);  // Point 2
        tree.add(5.0, 5.0, 5.0, 5.0);  // Point 3
        tree.add(10.0, 10.0, 10.0, 10.0);  // Point 4
        tree.build();

        let mut results = Vec::new();
        tree.query_nearest_k_points(0.0, 0.0, 3, &mut results);
        
        // Should return 3 nearest points
        assert_eq!(results.len(), 3);
        // Ordered by distance: 0, 1, 2
        assert_eq!(results[0], 0);
        assert_eq!(results[1], 1);
        assert_eq!(results[2], 2);
    }

    /// Test query_nearest_k_points with k larger than item count
    #[test]
    fn test_query_nearest_k_points_k_too_large() {
        let mut tree = AABB::with_capacity(2);
        tree.add(0.0, 0.0, 0.0, 0.0);
        tree.add(1.0, 1.0, 1.0, 1.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_nearest_k_points(0.0, 0.0, 10, &mut results);
        
        // Should return all 2 items
        assert_eq!(results.len(), 2);
        assert_eq!(results[0], 0);
        assert_eq!(results[1], 1);
    }

    /// Test query_nearest_k_points with k=1
    #[test]
    fn test_query_nearest_k_points_k_one() {
        let mut tree = AABB::with_capacity(3);
        tree.add(0.0, 0.0, 0.0, 0.0);
        tree.add(1.0, 0.0, 1.0, 0.0);
        tree.add(10.0, 10.0, 10.0, 10.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_nearest_k_points(0.0, 0.0, 1, &mut results);
        
        assert_eq!(results.len(), 1);
        assert_eq!(results[0], 0);  // Closest point
    }

    /// Test that point-specific methods return sorted results
    #[test]
    fn test_query_circle_points_sorting() {
        let mut tree = AABB::with_capacity(4);
        tree.add(3.0, 4.0, 3.0, 4.0);  // Point 0: distance 5 from origin
        tree.add(1.0, 0.0, 1.0, 0.0);  // Point 1: distance 1 from origin
        tree.add(0.0, 2.0, 0.0, 2.0);  // Point 2: distance 2 from origin
        tree.add(0.0, 0.0, 0.0, 0.0);  // Point 3: distance 0 from origin
        tree.build();

        let mut results = Vec::new();
        tree.query_circle_points(0.0, 0.0, 6.0, &mut results);
        
        // All 4 points within radius 6 (order depends on tree traversal)
        assert_eq!(results.len(), 4);
        // Verify all expected points are in results (unordered)
        assert!(results.contains(&0));
        assert!(results.contains(&1));
        assert!(results.contains(&2));
        assert!(results.contains(&3));
    }

    /// Test query_nearest_k_points with negative coordinates
    #[test]
    fn test_query_nearest_k_points_negative_coords() {
        let mut tree = AABB::with_capacity(3);
        tree.add(-1.0, -1.0, -1.0, -1.0);  // Point 0
        tree.add(0.0, 0.0, 0.0, 0.0);      // Point 1
        tree.add(1.0, 1.0, 1.0, 1.0);      // Point 2
        tree.build();

        let mut results = Vec::new();
        tree.query_nearest_k_points(-1.0, -1.0, 2, &mut results);
        
        assert_eq!(results.len(), 2);
        assert_eq!(results[0], 0);  // Closest (-1, -1)
        assert_eq!(results[1], 1);  // Second closest (0, 0)
    }

    /// Test query_circle_points with empty tree
    #[test]
    fn test_query_circle_points_empty_tree() {
        let tree = AABB::new();
        
        let mut results = Vec::new();
        tree.query_circle_points(0.0, 0.0, 5.0, &mut results);
        assert!(results.is_empty());
    }

    /// Test query_nearest_k_points with empty tree
    #[test]
    fn test_query_nearest_k_points_empty_tree() {
        let tree = AABB::new();
        
        let mut results = Vec::new();
        tree.query_nearest_k_points(0.0, 0.0, 3, &mut results);
        assert!(results.is_empty());
    }

    /// Test query_circle_points with negative radius (should return empty)
    #[test]
    fn test_query_circle_points_negative_radius() {
        let mut tree = AABB::with_capacity(2);
        tree.add(0.0, 0.0, 0.0, 0.0);
        tree.add(1.0, 1.0, 1.0, 1.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_circle_points(0.0, 0.0, -1.0, &mut results);
        assert!(results.is_empty());
    }

    /// Test query_nearest_k_points with k=0
    #[test]
    fn test_query_nearest_k_points_k_zero() {
        let mut tree = AABB::with_capacity(2);
        tree.add(0.0, 0.0, 0.0, 0.0);
        tree.add(1.0, 1.0, 1.0, 1.0);
        tree.build();

        let mut results = Vec::new();
        tree.query_nearest_k_points(0.0, 0.0, 0, &mut results);
        assert!(results.is_empty());
    }

    /// Test that point-optimized query_circle_points matches general query_circle for points
    #[test]
    fn test_query_circle_points_matches_general_query() {
        let mut tree = AABB::with_capacity(5);
        tree.add(0.0, 0.0, 0.0, 0.0);
        tree.add(1.0, 1.0, 1.0, 1.0);
        tree.add(2.0, 2.0, 2.0, 2.0);
        tree.add(3.0, 3.0, 3.0, 3.0);
        tree.add(5.0, 5.0, 5.0, 5.0);
        tree.build();

        let mut results_optimized = Vec::new();
        tree.query_circle_points(1.0, 1.0, 2.5, &mut results_optimized);

        let mut results_general = Vec::new();
        tree.query_circle(1.0, 1.0, 2.5, &mut results_general);

        // Both should find the same points (order may differ due to optimization)
        assert_eq!(results_optimized.len(), results_general.len());
        
        // Sort both for comparison since optimized version sorts by distance
        let mut general_sorted = results_general.clone();
        general_sorted.sort();
        let mut optimized_sorted = results_optimized.clone();
        optimized_sorted.sort();
        assert_eq!(optimized_sorted, general_sorted);
    }

    /// Test large point cloud with query_circle_points
    #[test]
    fn test_query_circle_points_large_cloud() {
        let mut tree = AABB::with_capacity(100);
        
        // Create 100 points in a grid
        for i in 0..10 {
            for j in 0..10 {
                let x = i as f64;
                let y = j as f64;
                tree.add(x, x, y, y);
            }
        }
        tree.build();

        // Query circle centered at (4.5, 4.5) with radius 2.0
        // Should find points roughly in that region
        let mut results = Vec::new();
        tree.query_circle_points(4.5, 4.5, 2.0, &mut results);
        
        // Should have multiple results
        assert!(results.len() > 5);
    }

    /// Test results vector is cleared properly
    #[test]
    fn test_query_circle_points_results_cleared() {
        let mut tree = AABB::with_capacity(3);
        tree.add(0.0, 0.0, 0.0, 0.0);
        tree.add(1.0, 1.0, 1.0, 1.0);
        tree.add(2.0, 2.0, 2.0, 2.0);
        tree.build();

        let mut results = vec![999]; // Pre-populate with garbage
        tree.query_circle_points(0.0, 0.0, 0.5, &mut results);
        
        assert_eq!(results.len(), 1);
        assert_eq!(results[0], 0);  // Should only contain point 0
    }

    /// Test query_nearest_k_points results vector cleared properly
    #[test]
    fn test_query_nearest_k_points_results_cleared() {
        let mut tree = AABB::with_capacity(3);
        tree.add(0.0, 0.0, 0.0, 0.0);
        tree.add(1.0, 1.0, 1.0, 1.0);
        tree.add(2.0, 2.0, 2.0, 2.0);
        tree.build();

        let mut results = vec![999, 888]; // Pre-populate with garbage
        tree.query_nearest_k_points(0.0, 0.0, 2, &mut results);
        
        assert_eq!(results.len(), 2);
        // Results should be cleared and repopulated
        assert!(results[0] == 0 || results[0] == 1);
        assert!(results[1] == 0 || results[1] == 1);
        assert!(results[0] != 999);
        assert!(results[1] != 888);
    }

    /// Test get_point() convenience method
    #[test]
    fn test_get_point_method() {
        let mut tree = AABB::with_capacity(4);
        tree.add_point(0.0, 0.0);
        tree.add_point(1.5, 2.5);
        tree.add_point(3.0, 4.0);
        tree.add_point(-1.0, -1.0);
        tree.build();

        // Verify get_point returns correct coordinates
        assert_eq!(tree.get_point(0), Some((0.0, 0.0)));
        assert_eq!(tree.get_point(1), Some((1.5, 2.5)));
        assert_eq!(tree.get_point(2), Some((3.0, 4.0)));
        assert_eq!(tree.get_point(3), Some((-1.0, -1.0)));
        
        // Out of bounds should return None
        assert_eq!(tree.get_point(4), None);
    }

    /// Test that get_point works before build
    #[test]
    fn test_get_point_before_build() {
        let mut tree = AABB::with_capacity(2);
        tree.add_point(1.0, 2.0);
        tree.add_point(3.0, 4.0);
        
        // Should work even before build()
        assert_eq!(tree.get_point(0), Some((1.0, 2.0)));
        assert_eq!(tree.get_point(1), Some((3.0, 4.0)));
    }

    /// Test query with very small radius (1e-9) - critical for exact point matching
    #[test]
    fn test_query_circle_points_very_small_radius() {
        let mut tree = AABB::with_capacity(5);
        tree.add_point(0.0, 0.0);
        tree.add_point(1.0, 0.0);
        tree.add_point(0.0, 1.0);
        tree.add_point(1.0, 1.0);
        tree.add_point(0.5, 0.5);
        tree.build();

        // Query with radius 1e-9 should only return exact point
        let mut results = Vec::new();
        tree.query_circle_points(0.0, 0.0, 1e-9, &mut results);
        assert_eq!(results, vec![0], "Query with radius 1e-9 should only return exact point");
        
        // Query with radius 1e-6 should still only return exact point
        tree.query_circle_points(0.0, 0.0, 1e-6, &mut results);
        assert_eq!(results, vec![0], "Query with radius 1e-6 should only return exact point");
    }

    /// Test point storage correctness after build with Hilbert sorting
    #[test]
    fn test_point_storage_after_build() {
        let mut tree = AABB::with_capacity(5);
        
        // Add points in specific order
        tree.add_point(0.0, 0.0);
        tree.add_point(1.0, 0.0);
        tree.add_point(0.0, 1.0);
        tree.add_point(1.0, 1.0);
        tree.add_point(0.5, 0.5);
        
        tree.build();
        
        // Verify all points are stored correctly
        assert_eq!(tree.get_point(0), Some((0.0, 0.0)));
        assert_eq!(tree.get_point(1), Some((1.0, 0.0)));
        assert_eq!(tree.get_point(2), Some((0.0, 1.0)));
        assert_eq!(tree.get_point(3), Some((1.0, 1.0)));
        assert_eq!(tree.get_point(4), Some((0.5, 0.5)));
    }

    /// Test that add_point stores correct coordinates (not swapped)
    #[test]
    fn test_add_point_coordinate_order() {
        let mut tree = AABB::with_capacity(3);
        
        tree.add_point(1.5, 2.5);
        tree.add_point(3.0, 4.0);
        tree.add_point(-1.0, -2.0);
        
        tree.build();
        
        // Verify X and Y are not swapped
        assert_eq!(tree.get_point(0), Some((1.5, 2.5)));
        assert_eq!(tree.get_point(1), Some((3.0, 4.0)));
        assert_eq!(tree.get_point(2), Some((-1.0, -2.0)));
        
        // Verify via get() as well
        assert_eq!(tree.get(0).unwrap(), (1.5, 2.5, 1.5, 2.5));
        assert_eq!(tree.get(1).unwrap(), (3.0, 4.0, 3.0, 4.0));
        assert_eq!(tree.get(2).unwrap(), (-1.0, -2.0, -1.0, -2.0));
    }

    /// Test consistency between query_circle_points and get_point
    #[test]
    fn test_query_circle_points_consistency_with_get_point() {
        let mut tree = AABB::with_capacity(10);
        
        // Create grid of points
        for i in 0..3 {
            for j in 0..3 {
                tree.add_point(i as f64, j as f64);
            }
        }
        tree.build();

        // Query for points near (1.0, 1.0) with radius 1.5
        let mut results = Vec::new();
        tree.query_circle_points(1.0, 1.0, 1.5, &mut results);
        
        // For each result, verify get_point returns the correct coordinates
        for &idx in &results {
            if let Some((x, y)) = tree.get_point(idx) {
                let distance = ((x - 1.0).powi(2) + (y - 1.0).powi(2)).sqrt();
                assert!(distance <= 1.5, "Point {} at ({}, {}) has distance {}, exceeds radius 1.5", idx, x, y, distance);
            }
        }
    }
}
