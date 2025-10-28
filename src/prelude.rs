//! Prelude module for convenient imports
//!
//! This module re-exports the most commonly used types from the crate.
//! Users can import everything they need with:
//!
//! ```
//! use aabb::prelude::*;
//! ```

#[doc(hidden)]
pub use crate::HilbertRTreeLeg;
pub use crate::HilbertRTree;
pub use crate::HilbertRTreeI32;

/// Convenient alias for `HilbertRTree` - floating-point coordinate spatial index
/// 
/// Allows using `AABB::new()` or `AABB::with_capacity(n)` instead of `HilbertRTree::new()`
pub type AABB = HilbertRTree;

/// Convenient alias for `HilbertRTreeI32` - memory-efficient i32-coordinate spatial index
/// 
/// Allows using `AABBI32::new()` or `AABBI32::with_capacity(n)` instead of `HilbertRTreeI32::new()`
pub type AABBI32 = HilbertRTreeI32;