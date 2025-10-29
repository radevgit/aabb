# Changelog

## [0.6.4] - 2025-10-29
- Multiple optimizations for query_intersecting() (43%)

## [0.6.3] - 2025-10-29
- Save and Load Hilbert Tree to file

## [0.6.2] - 2025-10-29
- Refactored all query methods to call `results.clear()` at entry point before early returns, ensuring consistent behavior and eliminating need for manual clearing between invocations
- Merged `src/queries.rs` into `src/hilbert_rtree_leg.rs` to consolidate all legacy code in a single file
- Added comprehensive assertions to all query examples (9 examples) with meaningful validation of query results
- Improved documentation with inline comments explaining expected query results
- All 137 unit tests passing; examples validated with assertions

## [0.6.1] - 2025-10-28
- Fix doctests

## [0.6.0] - 2025-10-28
- Significant performance optimizations on nearest_k and intersect_k methods.
- merge of some methods.
- added benchmarks

## [0.5.0] - 2025-10-28
- Implemented methods for i32 types

## [0.4.0] - 2025-10-28
- Refactor query method names for semantic clarity.
- Extend query methods documentation and examples.

## [0.3.0] - 2025-10-27
- Change HilbertRTree implementation to use byte array index.

## [0.2.0] - 2025-10-20
- Initial HilbertRTree implementation with 8 core query methods.
- Add Hilbert curve space-filling ordering for spatial locality.

## [0.1.0] - 2025-10-15
- Initial project scaffold with basic infrastructure. 