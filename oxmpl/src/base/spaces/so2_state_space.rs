// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use rand::Rng;
use std::f64::consts::PI;

use crate::base::{
    error::{StateSamplingError, StateSpaceError},
    space::StateSpace,
    state::SO2State,
};

/// A state space representing 2D rotations (SO(2)).
///
/// This space handles the circular nature of angles, ensuring that distance and interpolation
/// calculations correctly use the shortest path.
#[derive(Clone)]
pub struct SO2StateSpace {
    /// The bounds of the space, as a `(min, max)` tuple.
    pub bounds: (f64, f64),

    longest_valid_segment_fraction: f64,
}

impl SO2StateSpace {
    /// Creates a new `SO2StateSpace`.
    ///
    /// If `bounds_option` is `None`, the space defaults to the full circle `[-PI, PI)`.
    /// Provided bounds are normalized to the `[-PI, PI)` range.
    ///
    /// # Arguments
    ///
    /// * `bounds_option` - An `Option` containing a `(min, max)` tuple for the bounds.
    ///   If `None`, the space defaults to the full circle `[-PI, PI)`.
    ///
    /// # Errors
    ///
    /// Returns `StateSpaceError::InvalidBound` if the provided `min` bound is greater
    /// than or equal to the `max` bound. Note that wrapped ranges (e.g., from `1.5` to `-1.5`)
    /// are not supported by this constructor and should be handled by a `StateValidityChecker`.
    ///
    /// # Examples
    ///
    /// ```
    /// use oxmpl::base::space::SO2StateSpace;
    /// use std::f64::consts::PI;
    ///
    /// // Create a space representing the full circle
    /// let full_circle_space = SO2StateSpace::new(None).unwrap();
    /// assert_eq!(full_circle_space.bounds, (-PI, PI));
    ///
    /// // Create a space bounded to the upper semi-circle
    /// let upper_half_space = SO2StateSpace::new(Some((0.0, PI))).unwrap();
    /// assert_eq!(upper_half_space.bounds, (0.0, PI));
    /// ```
    pub fn new(bounds_option: Option<(f64, f64)>) -> Result<Self, StateSpaceError> {
        let bounds = bounds_option.unwrap_or((-PI, PI));

        if bounds.0 >= bounds.1 {
            return Err(StateSpaceError::InvalidBound {
                lower: bounds.0,
                upper: bounds.1,
            });
        }

        let clamped_bounds = (bounds.0.max(-PI), bounds.1.min(PI));

        Ok(Self {
            bounds: clamped_bounds,
            longest_valid_segment_fraction: 0.05,
        })
    }

    /// Returns the maximum possible distance in this space, which is always PI.
    pub fn get_maximum_extent(&self) -> f64 {
        PI
    }

    /// Sets the fraction used to determine motion checking resolution.
    pub fn set_longest_valid_segment_fraction(&mut self, fraction: f64) {
        if fraction > 0.0 && fraction <= 1.0 {
            self.longest_valid_segment_fraction = fraction;
        } else if fraction <= 0.0 {
            self.longest_valid_segment_fraction = 0.;
        } else {
            self.longest_valid_segment_fraction = 1.;
        }
    }
}

impl StateSpace for SO2StateSpace {
    type StateType = SO2State;

    /// Computes the shortest angle (in radians) between two states.
    fn distance(&self, state1: &Self::StateType, state2: &Self::StateType) -> f64 {
        let mut diff = state1.value - state2.value;
        diff = (diff + PI).rem_euclid(2.0 * PI) - PI;
        diff.abs()
    }

    /// Performs linear interpolation between two states. Also normalises the result.
    ///
    /// The resulting state's components are calculated as:
    /// `out_state.values[i] = from.values[i] + t * (to.values[i] - from.values[i])`
    fn interpolate(
        &self,
        from: &Self::StateType,
        to: &Self::StateType,
        t: f64,
        out_state: &mut Self::StateType,
    ) {
        let mut diff_to_from = to.clone().normalise().value - from.clone().normalise().value;
        if diff_to_from > PI {
            diff_to_from -= 2.0 * PI;
        } else if diff_to_from < -PI {
            diff_to_from += 2.0 * PI;
        }
        out_state.value = from.value + diff_to_from * t;
        out_state.value = out_state.normalise().value;
    }

    /// Modifies the state by clamping each of its values to the space's bounds.
    fn enforce_bounds(&self, state: &mut Self::StateType) {
        state.normalise();

        if self.satisfies_bounds(state) {
            return;
        };

        let (min_b, max_b) = self.bounds;
        let dist_to_min = self.distance(&SO2State { value: min_b }, state);
        let dist_to_max = self.distance(&SO2State { value: max_b }, state);

        if dist_to_min < dist_to_max {
            state.value = min_b;
        } else {
            state.value = max_b;
        }
    }

    /// Checks if a state is within the defined angular bounds.
    fn satisfies_bounds(&self, state: &Self::StateType) -> bool {
        let val = state.clone().normalise().value;
        let (lower, upper) = self.bounds;
        val >= lower && val <= upper
    }

    /// Generates a random angle from within the defined bounds.
    ///
    /// # Arguments
    ///
    /// * `rng` - A mutable reference to a random number generator.
    ///
    /// # Errors
    ///
    /// This implementation will not return an error, as the constructor ensures
    /// the bounds are always valid for sampling. The `Result` is returned to
    /// satisfy the `StateSpace` trait.
    fn sample_uniform(&self, rng: &mut impl Rng) -> Result<SO2State, StateSamplingError> {
        let (lower, upper) = self.bounds;
        Ok(SO2State {
            value: rng.random_range(lower..upper),
        })
    }

    fn get_longest_valid_segment_length(&self) -> f64 {
        self.get_maximum_extent() * self.longest_valid_segment_fraction
    }
}
