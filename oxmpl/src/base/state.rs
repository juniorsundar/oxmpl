// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

pub use crate::base::states::so2_state::SO2State;

/// A marker trait for all state types in the planning library.
///
/// A `State` represents a single point, configuration, or snapshot of the system
/// being planned for.
///
/// Supertrait bounds:
/// - `Clone`: States must be copyable.
/// - `Send` + `Sync`: States must be safe to share and send between threads.
/// - `'static`: The state type does not contain any non-static references.
pub trait State: Clone {}

/// A state representing a point in an N-dimensional Euclidean space (R^n).
#[derive(Clone, Debug, PartialEq)]
pub struct RealVectorState {
    /// Values of each dimension of the state.
    pub values: Vec<f64>,
}
impl RealVectorState {
    /// Creates a new `RealVectorState`.
    pub fn new(vals: Vec<f64>) -> Self {
        RealVectorState { values: vals }
    }
}
/// Implements the `State` marker trait for `RealVectorState`.
impl State for RealVectorState {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_real_vector_state_creation() {
        let state = RealVectorState {
            values: vec![1.0, 2.0],
        };
        assert_eq!(state.values, vec![1.0, 2.0]);
    }

    #[test]
    fn test_real_vector_state_clone() {
        let state1 = RealVectorState {
            values: vec![1.0, 2.0],
        };
        let state2 = state1.clone();
        assert_eq!(state1, state2);
    }
}
