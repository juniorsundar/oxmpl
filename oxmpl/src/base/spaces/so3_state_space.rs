// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use rand::Rng;
use std::f64::consts::PI;

use crate::base::{
    error::{StateSamplingError, StateSpaceError},
    space::StateSpace,
    state::SO3State,
};

/// A state space representing 3D rotations (the Special Orthogonal group SO(3)).
///
/// States are represented by unit quaternions.
#[derive(Clone)]
pub struct SO3StateSpace {
    /// The bounds of the space, as a `(center_rotation, max_angle)` tuple.
    pub bounds: (SO3State, f64),

    longest_valid_segment_fraction: f64,
}

impl SO3StateSpace {
    /// Creates a new `SO3StateSpace`.
    ///
    /// # Arguments
    ///
    /// * `bounds_option` - An optional tuple `(center_rotation, max_angle)` where `max_angle` is
    ///   the maximum allowed angular distance from the center in radians.
    ///   - If `None`, the space is considered "unbounded": the center is the identity rotation and
    ///     the `max_angle` is set to `PI`, covering all possible rotations.
    ///   - If `Some`, the provided bounds are validated.
    ///
    /// # Errors
    ///
    /// * `StateSpaceError::InvalidAngularDistance` if the provided `max_angle` is negative.
    ///
    /// # Examples
    ///
    /// ```
    /// use oxmpl::base::space::SO3StateSpace;
    /// use oxmpl::base::state::SO3State;
    /// use std::f64::consts::PI;
    ///
    /// // Create an unbounded space (covers all rotations)
    /// let unbounded_space = SO3StateSpace::new(None).unwrap();
    /// assert_eq!(unbounded_space.bounds.0, SO3State::identity());
    /// assert_eq!(unbounded_space.bounds.1, PI);
    ///
    /// // Create a space bounded to a 30-degree cone around the identity rotation
    /// let center = SO3State::identity();
    /// let max_angle = 30.0f64.to_radians();
    /// let bounded_space = SO3StateSpace::new(Some((center, max_angle))).unwrap();
    /// ```
    pub fn new(bounds_option: Option<(SO3State, f64)>) -> Result<Self, StateSpaceError> {
        let bounds = match bounds_option {
            Some((center_rotation, max_angle)) => {
                if max_angle < 0.0 {
                    return Err(StateSpaceError::InvalidAngularDistance { lower: max_angle });
                }

                let clamped_max_angle = max_angle.min(PI);

                (center_rotation, clamped_max_angle)
            }
            None => {
                let center = SO3State::identity();
                let max_angle = PI;
                (center, max_angle)
            }
        };

        Ok(Self {
            bounds,
            longest_valid_segment_fraction: 0.05,
        })
    }

    /// Returns the maximum possible distance in this space, which is always 0.5*PI.
    pub fn get_maximum_extent(&self) -> f64 {
        0.5 * PI
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

impl StateSpace for SO3StateSpace {
    type StateType = SO3State;

    /// Computes the shortest angle between two rotations using the quaternion dot product.
    fn distance(&self, state1: &Self::StateType, state2: &Self::StateType) -> f64 {
        let abs_dot =
            (state1.x * state2.x + state1.y * state2.y + state1.z * state2.z + state1.w * state2.w)
                .abs();
        let clamped_dot = abs_dot.min(1.0);
        2.0 * clamped_dot.acos()
    }

    /// Performs Spherical Linear Interpolation (SLERP) between two states.
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
        let mut dot = from.x * to.x + from.y * to.y + from.z * to.z + from.w * to.w;

        let sign = if dot < 0.0 { -1.0 } else { 1.0 };
        dot *= sign;

        const DOT_THRESHOLD: f64 = 0.9995;
        if dot > DOT_THRESHOLD {
            // LERP
            out_state.x = from.x + t * (to.x * sign - from.x);
            out_state.y = from.y + t * (to.y * sign - from.y);
            out_state.z = from.z + t * (to.z * sign - from.z);
            out_state.w = from.w + t * (to.w * sign - from.w);

            let norm = (out_state.x.powi(2)
                + out_state.y.powi(2)
                + out_state.z.powi(2)
                + out_state.w.powi(2))
            .sqrt();
            out_state.x /= norm;
            out_state.y /= norm;
            out_state.z /= norm;
            out_state.w /= norm;
        } else {
            // SLERP
            let theta = dot.acos();
            let sin_theta = theta.sin();

            let s0 = ((1.0 - t) * theta).sin() / sin_theta;
            let s1 = (t * theta).sin() / sin_theta * sign;

            out_state.x = from.x * s0 + to.x * s1;
            out_state.y = from.y * s0 + to.y * s1;
            out_state.z = from.z * s0 + to.z * s1;
            out_state.w = from.w * s0 + to.w * s1;
        }
    }

    /// Projects a state onto the boundary of the valid "cone of freedom" if it is out of bounds.
    fn enforce_bounds(&self, state: &mut Self::StateType) {
        match state.normalise() {
            Ok(norm) => *state = norm,
            Err(_) => *state = SO3State::identity(),
        };

        if self.satisfies_bounds(state) {
            return;
        };

        let (center_rotation, max_angle) = &self.bounds;
        let actual_distance = self.distance(center_rotation, state);
        if actual_distance < 1e-9 {
            return;
        }

        let t = *max_angle / actual_distance;
        let original_state = state.clone();
        self.interpolate(center_rotation, &original_state, t, state);
    }

    /// Checks if a state is within the defined "cone of freedom" bounds.
    fn satisfies_bounds(&self, state: &Self::StateType) -> bool {
        let (center_rotation, max_angle) = &self.bounds;
        let deviation = self.distance(center_rotation, state);

        deviation <= *max_angle
    }

    /// Generates a uniformly random rotation within the defined bounds.
    ///
    /// # Arguments
    ///
    /// * `rng` - A mutable reference to a random number generator.
    ///
    /// # Errors
    ///
    /// This function will not return an error, as the constructor ensures the bounds are always
    /// valid. The `Result` is returned to satisfy the `StateSpace` trait.
    fn sample_uniform(&self, rng: &mut impl Rng) -> Result<SO3State, StateSamplingError> {
        let (center_rotation, max_angle) = &self.bounds;

        if *max_angle < 1e-9 {
            return Ok(center_rotation.clone());
        }

        // The rejection sampling
        loop {
            let x: f64 = rng.random_range(-1.0..1.0);
            let y: f64 = rng.random_range(-1.0..1.0);
            let z: f64 = rng.random_range(-1.0..1.0);
            let w: f64 = rng.random_range(-1.0..1.0);

            let norm_sq = x * x + y * y + z * z + w * w;

            if norm_sq > 1e-9 && norm_sq < 1.0 {
                let norm = norm_sq.sqrt();
                let random_quat = SO3State {
                    x: x / norm,
                    y: y / norm,
                    z: z / norm,
                    w: w / norm,
                };

                let distance = self.distance(center_rotation, &random_quat);
                if distance <= *max_angle {
                    return Ok(random_quat);
                }
            }
        }
    }

    fn get_longest_valid_segment_length(&self) -> f64 {
        self.get_maximum_extent() * self.longest_valid_segment_fraction
    }
}
