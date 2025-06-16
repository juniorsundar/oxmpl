// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::{error::StateSamplingError, state::State};
use rand::Rng;

/// The base trait for all goal definitions.
///
/// A `Goal` represents the termination condition for a planner. This trait is the minimum
/// requirement for any goal.
pub trait Goal<S: State> {
    /// Returns `true` if the given state satisfies the goal condition.
    fn is_satisfied(&self, state: &S) -> bool;
}

/// A `Goal` that is also a region
///
/// `Goal<S>` makes this a **subtrait** of `Goal`. Any type that implements `GoalRegion` must also
/// implement `Goal`.
pub trait GoalRegion<S: State>: Goal<S> {
    /// Returns the distance from a state to the goal region.
    ///
    /// If the state is inside the goal region, the distance should be `0.0`.
    fn distance_goal(&self, state: &S) -> f64;
}

/// A `GoalRegion` that can be sampled directly.
///
/// It is a subtrait of `GoalRegion`, so any implementor must also provide `distance_goal` and
/// `is_satisfied` methods.
pub trait GoalSampleableRegion<S: State>: GoalRegion<S> {
    /// Generates a state from within the goal region.
    ///
    /// This allows a planner to directly sample configurations that are known to
    /// be valid goal states.
    ///
    /// # Errors
    /// Can return an error if sampling is not possible.
    fn sample_goal(&self, rng: &mut impl Rng) -> Result<S, StateSamplingError>;
}
