// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::{
    error::StateSpaceError,
    space::{RealVectorStateSpace, SO2StateSpace, StateSpace},
    state::SE2State,
};

/// A fixed typed state space for planar rigid-body transformations (SE(2)).
///
/// This space owns a 2D translational space and a planar rotational space directly instead of
/// routing normal SE(2) operations through the dynamic compound-state machinery.
#[derive(Clone)]
pub struct SE2StateSpace {
    translation_space: RealVectorStateSpace,
    rotation_space: SO2StateSpace,
    rotation_weight: f64,
}

impl SE2StateSpace {
    /// Creates a new `SE2StateSpace`.
    ///
    /// The `bounds_option` allows specifying the valid range for the state. If provided, it must
    /// be a `Vec` containing exactly three `(min, max)` tuples, corresponding to the bounds for x,
    /// y, and yaw, respectively.
    ///
    /// The `weight` parameter is applied to the rotational component (yaw) when calculating
    /// distances, to control the trade-off between translational and rotational costs.
    ///
    /// # Examples
    ///
    /// ```
    /// use oxmpl::base::{space::SE2StateSpace, error::StateSpaceError};
    ///
    /// let space = SE2StateSpace::new(0.5, None).unwrap();
    ///
    /// let bounds = vec![(-1.0, 1.0), (-2.0, 2.0), (-3.14, 3.14)];
    /// let bounded_space = SE2StateSpace::new(1.0, Some(bounds)).unwrap();
    ///
    /// let invalid_bounds = vec![(-1.0, 1.0)];
    /// let result = SE2StateSpace::new(1.0, Some(invalid_bounds));
    /// assert!(matches!(result, Err(StateSpaceError::DimensionMismatch { .. })));
    /// ```
    pub fn new(
        weight: f64,
        bounds_option: Option<Vec<(f64, f64)>>,
    ) -> Result<Self, StateSpaceError> {
        let (translation_space, rotation_space) = match bounds_option {
            Some(bounds) => {
                if bounds.len() != 3 {
                    return Err(StateSpaceError::DimensionMismatch {
                        expected: 3,
                        found: bounds.len(),
                    });
                }

                (
                    RealVectorStateSpace::new(2, Some(vec![bounds[0], bounds[1]]))?,
                    SO2StateSpace::new(Some(bounds[2]))?,
                )
            }
            None => (
                RealVectorStateSpace::new(2, None)?,
                SO2StateSpace::new(None)?,
            ),
        };

        Ok(Self {
            translation_space,
            rotation_space,
            rotation_weight: weight,
        })
    }
}

impl StateSpace for SE2StateSpace {
    type StateType = SE2State;

    fn distance(&self, state1: &Self::StateType, state2: &Self::StateType) -> f64 {
        let translation_distance = self
            .translation_space
            .distance(state1.get_translation(), state2.get_translation());
        let rotation_distance = self
            .rotation_space
            .distance(state1.get_rotation(), state2.get_rotation());

        (translation_distance.powi(2) + (self.rotation_weight * rotation_distance).powi(2)).sqrt()
    }

    fn interpolate(
        &self,
        from: &Self::StateType,
        to: &Self::StateType,
        t: f64,
        state: &mut Self::StateType,
    ) {
        self.translation_space.interpolate(
            from.get_translation(),
            to.get_translation(),
            t,
            state.translation_mut(),
        );
        self.rotation_space.interpolate(
            from.get_rotation(),
            to.get_rotation(),
            t,
            state.rotation_mut(),
        );
    }

    fn enforce_bounds(&self, state: &mut Self::StateType) {
        self.translation_space
            .enforce_bounds(state.translation_mut());
        self.rotation_space.enforce_bounds(state.rotation_mut());
    }

    fn satisfies_bounds(&self, state: &Self::StateType) -> bool {
        self.translation_space
            .satisfies_bounds(state.get_translation())
            && self.rotation_space.satisfies_bounds(state.get_rotation())
    }

    fn sample_uniform(
        &self,
        rng: &mut impl rand::Rng,
    ) -> Result<Self::StateType, crate::base::error::StateSamplingError> {
        let translation = self.translation_space.sample_uniform(rng)?;
        let rotation = self.rotation_space.sample_uniform(rng)?;
        Ok(SE2State::from_parts(translation, rotation))
    }

    fn get_longest_valid_segment_length(&self) -> f64 {
        let translation_segment = self.translation_space.get_longest_valid_segment_length();
        let rotation_segment = self.rotation_space.get_longest_valid_segment_length();

        (translation_segment.powi(2) + (self.rotation_weight * rotation_segment).powi(2)).sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::base::state::SE2State;
    use rand::rng;
    use std::f64::consts::PI;

    #[test]
    fn test_se2_space_creation() {
        assert!(SE2StateSpace::new(0.5, None).is_ok());

        let bounds = vec![(-1.0, 1.0), (-1.0, 1.0), (-PI, PI)];
        assert!(SE2StateSpace::new(1.0, Some(bounds)).is_ok());
    }

    #[test]
    fn test_se2_space_creation_invalid_bounds() {
        let bounds = vec![(-1.0, 1.0)];
        let result = SE2StateSpace::new(1.0, Some(bounds));
        assert!(result.is_err());
        match result {
            Err(StateSpaceError::DimensionMismatch { expected, found }) => {
                assert_eq!(expected, 3);
                assert_eq!(found, 1);
            }
            _ => panic!("Expected DimensionMismatch error"),
        }
    }

    #[test]
    fn test_fixed_typed_se2_public_behavior() {
        let bounds = vec![(-1.0, 1.0), (-2.0, 2.0), (-PI / 2.0, PI / 2.0)];
        let space = SE2StateSpace::new(0.5, Some(bounds)).unwrap();
        let state1 = SE2State::new(0.0, 0.0, 0.0);
        let state2 = SE2State::new(3.0, 4.0, 3.0 * PI);
        let mut candidate = SE2State::new(2.0, -3.0, 0.8 * PI);

        let expected_distance = (5.0f64.powi(2) + (0.5 * PI).powi(2)).sqrt();
        assert!((space.distance(&state1, &state2) - expected_distance).abs() < 1e-9);
        assert!((state2.get_yaw() + PI).abs() < 1e-9);
        assert!(!space.satisfies_bounds(&candidate));

        space.enforce_bounds(&mut candidate);
        assert_eq!(candidate.get_x(), 1.0);
        assert_eq!(candidate.get_y(), -2.0);
        assert!((candidate.get_yaw() - (PI / 2.0)).abs() < 1e-9);
    }

    #[test]
    fn test_distance() {
        let space = SE2StateSpace::new(0.5, None).unwrap();
        let state1 = SE2State::new(0.0, 0.0, 0.0);
        let state2 = SE2State::new(3.0, 4.0, PI);

        let expected_dist_r2: f64 = 5.0;
        let expected_dist_so2 = PI;
        let expected_total_dist =
            (expected_dist_r2.powi(2) + (0.5 * expected_dist_so2).powi(2)).sqrt();

        assert!((space.distance(&state1, &state2) - expected_total_dist).abs() < 1e-9);
        assert_eq!(space.distance(&state1, &state1), 0.0);
    }

    #[test]
    fn test_interpolate() {
        let space = SE2StateSpace::new(1.0, None).unwrap();
        let state1 = SE2State::new(0.0, 0.0, 0.0);
        let state2 = SE2State::new(10.0, -10.0, PI / 2.0);
        let mut interpolated_state = SE2State::new(0.0, 0.0, 0.0);

        space.interpolate(&state1, &state2, 0.5, &mut interpolated_state);

        assert_eq!(interpolated_state.get_x(), 5.0);
        assert_eq!(interpolated_state.get_y(), -5.0);
        assert!((interpolated_state.get_yaw() - PI / 4.0).abs() < 1e-9);

        space.interpolate(&state1, &state2, 0.0, &mut interpolated_state);
        assert_eq!(interpolated_state.get_x(), state1.get_x());
        assert_eq!(interpolated_state.get_y(), state1.get_y());
        assert_eq!(interpolated_state.get_yaw(), state1.get_yaw());

        space.interpolate(&state1, &state2, 1.0, &mut interpolated_state);
        assert_eq!(interpolated_state.get_x(), state2.get_x());
        assert_eq!(interpolated_state.get_y(), state2.get_y());
        assert_eq!(interpolated_state.get_yaw(), state2.get_yaw());
    }

    #[test]
    fn test_bounds() {
        let bounds = vec![(-1.0, 1.0), (-2.0, 2.0), (-PI / 2.0, PI / 2.0)];
        let space = SE2StateSpace::new(1.0, Some(bounds)).unwrap();

        let out_of_bounds_x = SE2State::new(2.0, 0.0, 0.0);
        assert!(!space.satisfies_bounds(&out_of_bounds_x));

        let out_of_bounds_y = SE2State::new(0.0, -3.0, 0.0);
        assert!(!space.satisfies_bounds(&out_of_bounds_y));

        let out_of_bounds_yaw = SE2State::new(0.0, 0.0, 0.8 * PI);
        assert!(!space.satisfies_bounds(&out_of_bounds_yaw));

        let mut out_of_bounds_state = SE2State::new(2.0, -3.0, 0.8 * PI);
        let in_bounds_state = SE2State::new(0.5, 1.5, 0.0);

        assert!(!space.satisfies_bounds(&out_of_bounds_state));
        assert!(space.satisfies_bounds(&in_bounds_state));

        space.enforce_bounds(&mut out_of_bounds_state);
        assert_eq!(out_of_bounds_state.get_x(), 1.0);
        assert_eq!(out_of_bounds_state.get_y(), -2.0);
        assert!((out_of_bounds_state.get_yaw() - (PI / 2.0)).abs() < 1e-9);
    }

    #[test]
    fn test_sample_uniform() {
        let bounds = vec![(-1.0, 1.0), (5.0, 10.0), (0.0, PI)];
        let space = SE2StateSpace::new(1.0, Some(bounds)).unwrap();
        let mut rng = rng();

        for _ in 0..100 {
            let sample = space.sample_uniform(&mut rng).unwrap();
            assert!(space.satisfies_bounds(&sample));
        }
    }
}
