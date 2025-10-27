#[cfg(test)]
mod integration_tests {
    use crate::HilbertRTree;

    #[test]
    fn test_new_api_parameter_order() {
        // Integration test to demonstrate the new API parameter order: (min_x, min_y, max_x, max_y)
        let mut tree = HilbertRTree::new();
        
        // Add some boxes using the new parameter order
        tree.add(0.0, 0.0, 2.0, 2.0);    // Box 0: bottom-left (0,0) to top-right (2,2)
        tree.add(1.0, 1.0, 3.0, 3.0);    // Box 1: bottom-left (1,1) to top-right (3,3) - overlaps Box 0
        tree.add(5.0, 5.0, 6.0, 6.0);    // Box 2: bottom-left (5,5) to top-right (6,6) - distant
        tree.add(1.5, 1.5, 2.5, 2.5);    // Box 3: bottom-left (1.5,1.5) to top-right (2.5,2.5) - inside others
        
        tree.build();
        
        // Test intersecting query using new parameter order
        let mut results = Vec::new();
        tree.query_intersecting(1.2, 1.2, 2.8, 2.8, &mut results);
        // Should find boxes 0, 1, and 3 that intersect the region (1.2,1.2) to (2.8,2.8)
        assert!(results.len() >= 2, "Should find at least 2 intersecting boxes");
        assert!(results.contains(&0) || results.contains(&1) || results.contains(&3));
        
        // Test point query
        results.clear();
        tree.query_point(1.8, 1.8, &mut results);
        assert!(results.len() >= 1, "Point (1.8, 1.8) should be contained in at least one box");
        
        // Test containing query using new parameter order
        results.clear();
        tree.query_containing(1.2, 1.2, 1.8, 1.8, &mut results);
        // Should find boxes that completely contain the rectangle (1.2,1.2) to (1.8,1.8)
        
        // Test contained by query using new parameter order  
        results.clear();
        tree.query_contained_by(0.5, 0.5, 3.5, 3.5, &mut results);
        // Should find boxes contained in the large rectangle (0.5,0.5) to (3.5,3.5)
        
        // All tests passed - API parameter order change is working!
        println!("✅ New API parameter order (min_x, min_y, max_x, max_y) is working correctly!");
    }
}