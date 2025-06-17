// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::state;

/// A trait for checking if states are valid.
///
/// This is the central trait for defining problem-specific constraints, most commonly
/// for collision checking. A `StateValidityChecker` is given to a planner, which then
/// uses the `is_valid` method to determine if sampled states or motions are allowed.
///
/// This generic approach allows users of the `oxmpl` library to define any custom
/// logic for state validity (e.g., checking against complex geometry, satisfying
/// physical constraints) without changing the core planners.
///
/// # Example
///
/// ```
/// use oxmpl::base::state::{State, RealVectorState};
/// use oxmpl::base::validity::StateValidityChecker;
///
/// // A simple checker that considers any state with a positive x-coordinate to be invalid.
/// struct PositiveXIsInvalidChecker;
///
/// impl StateValidityChecker<RealVectorState> for PositiveXIsInvalidChecker {
///     fn is_valid(&self, state: &RealVectorState) -> bool {
///         state.values.get(0).map_or(true, |&x| x <= 0.0)
///     }
/// }
///
/// let checker = PositiveXIsInvalidChecker;
/// let valid_state = RealVectorState { values: vec![-1.0, 5.0] };
/// let invalid_state = RealVectorState { values: vec![1.0, 5.0] };
///
/// assert!(checker.is_valid(&valid_state));
/// assert!(!checker.is_valid(&invalid_state));
/// ```
pub trait StateValidityChecker<S: state::State> {
    /// Checks if the given state is valid.
    ///
    /// # Parameters
    /// * `state` - A reference to the state to be checked.
    ///
    /// # Returns
    /// Returns `true` if the state is valid, and `false` otherwise.
    fn is_valid(&self, state: &S) -> bool;
}
