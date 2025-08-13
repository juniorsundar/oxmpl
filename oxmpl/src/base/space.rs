// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use rand::Rng;

pub use crate::base::spaces::{
    real_vector_state_space::RealVectorStateSpace, so2_state_space::SO2StateSpace,
};
use crate::base::{error::StateSamplingError, state::State};

/// Defines a space in which planning can be performed.
///
/// A `StateSpace` represents the manifold where states exist. It defines the properties and
/// operations applicable to that space as a whole, such as how to measure distance, how to
/// interpolate between states, and how to generate new states.
///
/// This trait is generic and can be implemented for various types of spaces, like N-dimensional
/// Euclidean vectors (`RealVectorStateSpace`) or 2D rotations (`SO2StateSpace`). Planners are
/// written to be generic over this trait, allowing them to solve problems in any space that
/// implements these fundamental operations.
///
/// # Examples
///
/// ```
/// use std::f64;
/// use oxmpl::base::state::State;
/// use oxmpl::base::space::StateSpace;
/// use oxmpl::base::error::StateSamplingError;
/// use rand::Rng;
///
/// #[derive(Debug, Clone, PartialEq)]
/// struct Point1D {
///     x: f64,
/// }
/// impl State for Point1D {}
///
/// struct LineSegmentSpace {
///     bounds: (f64, f64),
/// }
///
/// impl StateSpace for LineSegmentSpace {
///     type StateType = Point1D;
///
///     fn distance(&self, state1: &Self::StateType, state2: &Self::StateType) -> f64 {
///         (state1.x - state2.x).abs()
///     }
///
///     fn interpolate(&self, from: &Self::StateType, to: &Self::StateType, t: f64, state: &mut Self::StateType) {
///         state.x = from.x + (to.x - from.x) * t;
///     }
///
///     fn enforce_bounds(&self, state: &mut Self::StateType) {
///         state.x = state.x.clamp(self.bounds.0, self.bounds.1);
///     }
///
///     fn satisfies_bounds(&self, state: &Self::StateType) -> bool {
///         state.x >= self.bounds.0 && state.x <= self.bounds.1
///     }
///
///     fn sample_uniform(&self, rng: &mut impl Rng) -> Result<Self::StateType, StateSamplingError> {
///         Ok(Point1D { x: rng.gen_range(self.bounds.0..self.bounds.1) })
///     }
///
///     fn get_longest_valid_segment_length(&self) -> f64 {
///         (self.bounds.1 - self.bounds.0) * 0.05
///     }
/// }
///
/// let space = LineSegmentSpace { bounds: (0.0, 10.0) };
/// let mut rng = rand::thread_rng();
/// let random_state = space.sample_uniform(&mut rng).unwrap();
///
/// assert!(space.satisfies_bounds(&random_state));
/// assert_eq!(space.get_longest_valid_segment_length(), 0.5);
/// ```
pub trait StateSpace {
    /// StateType defines what is acceptable in current StateSpace
    type StateType: State;

    /// Find distance between current state1 and target state2.
    ///
    /// The distance metric is specific to the topology of the space. For example, a
    /// `RealVectorStateSpace` would use Euclidean distance, while an `SO2StateSpace` would compute
    /// the shortest angle on a circle.
    ///
    /// # Parameters
    /// * `state1` - The first state.
    /// * `state2` - The second state.
    fn distance(&self, state1: &Self::StateType, state2: &Self::StateType) -> f64;

    /// Find state interpolated between `from` and `to` states given 0<=`t`<=1.
    ///
    /// The resulting state is a point on the path between `from` and `to`, determined by the
    /// interpolation parameter `t`. The path is assumed to be a straight line.
    ///
    /// # Parameters
    /// * `from` - The starting state for interpolation.
    /// * `to` - The ending state for interpolation.
    /// * `t` - The interpolation factor.
    /// * `state` - A mutable reference to a state that will be updated with the result.
    fn interpolate(
        &self,
        from: &Self::StateType,
        to: &Self::StateType,
        t: f64,
        state: &mut Self::StateType,
    );

    /// Modifies the given state to ensure it conforms to the space's defined bounds.
    ///
    /// A `RealVectorStateSpace` might clamp the values of the state to its min/max bounds.
    ///
    /// While an `SO2StateSpace` would normalise an angle to range (e.g., `[-PI, PI)`). This method
    /// modifies the state in-place.
    fn enforce_bounds(&self, state: &mut Self::StateType);

    /// Checks if a state is within the valid bounds of this space.
    ///
    /// This method only checks against the fundamental boundaries of the space definition. It does
    /// *not* check for things like collisions, which is the job of a `StateValidityChecker`.
    ///
    /// # Returns
    /// Returns `true` if the state is within bounds, `false` otherwise.
    fn satisfies_bounds(&self, state: &Self::StateType) -> bool;

    /// Generates a state uniformly at random from the entire state space.
    ///
    /// This method relies on the space having well-defined, finite bounds.
    ///
    /// # Parameters
    /// * `rng` - A mutable reference to a random number generator.
    ///
    /// # Errors
    /// Returns a `StateSamplingError::UnboundedDimension` if the space is unbounded
    /// in any dimension, as uniform sampling from an infinite domain is not possible.
    fn sample_uniform(&self, rng: &mut impl Rng) -> Result<Self::StateType, StateSamplingError>;

    /// Gets the length of the longest segment that can be assumed valid.
    ///
    /// This is a heuristic used to determine the resolution for motion validation. A smaller value
    /// means motions are checked more frequently.
    fn get_longest_valid_segment_length(&self) -> f64;
}
