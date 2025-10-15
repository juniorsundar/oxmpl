// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::{
    error::StateSpaceError,
    space::{AnyStateSpace, CompoundStateSpace, RealVectorStateSpace, SO2StateSpace, StateSpace},
    state::SE2State,
};

/// A state space for 2D rigid body transformations (SE(2)).
///
/// This space combines a 2D translational space (`RealVectorStateSpace` of dimension 2) and a 2D
/// rotational space (`SO2StateSpace`). It allows for defining bounds for x, y, and yaw, and
/// calculating weighted distances between states.
#[derive(Clone)]
pub struct SE2StateSpace(pub CompoundStateSpace);

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
        let (r2, so2) = match bounds_option {
            Some(bounds) => {
                if bounds.len() != 3 {
                    return Err(StateSpaceError::DimensionMismatch {
                        expected: 3,
                        found: bounds.len(),
                    });
                } else {
                    (
                        RealVectorStateSpace::new(2, Some(vec![bounds[0], bounds[1]]))?,
                        SO2StateSpace::new(Some(bounds[2]))?,
                    )
                }
            }
            None => (
                RealVectorStateSpace::new(2, None)?,
                SO2StateSpace::new(None)?,
            ),
        };

        let compound_space =
            CompoundStateSpace::new(vec![Box::new(r2), Box::new(so2)], vec![1.0, weight]);
        Ok(SE2StateSpace(compound_space))
    }
}

impl StateSpace for SE2StateSpace {
    type StateType = SE2State;

    fn distance(&self, state1: &Self::StateType, state2: &Self::StateType) -> f64 {
        self.0.distance_dyn(&state1.0, &state2.0)
    }

    fn interpolate(
        &self,
        from: &Self::StateType,
        to: &Self::StateType,
        t: f64,
        state: &mut Self::StateType,
    ) {
        self.0.interpolate_dyn(&from.0, &to.0, t, &mut state.0);
    }

    fn enforce_bounds(&self, state: &mut Self::StateType) {
        self.0.enforce_bounds_dyn(&mut state.0);
    }

    fn satisfies_bounds(&self, state: &Self::StateType) -> bool {
        self.0.satisfies_bounds_dyn(&state.0)
    }

    fn sample_uniform(
        &self,
        rng: &mut impl rand::Rng,
    ) -> Result<Self::StateType, crate::base::error::StateSamplingError> {
        let compound_state = self.0.sample_uniform(rng)?;
        Ok(SE2State(compound_state))
    }

    fn get_longest_valid_segment_length(&self) -> f64 {
        self.0.get_longest_valid_segment_length_dyn()
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
    fn test_distance() {
        let space = SE2StateSpace::new(0.5, None).unwrap();
        let state1 = SE2State::new(0.0, 0.0, 0.0);
        let state2 = SE2State::new(3.0, 4.0, PI);

        let expected_dist_r2: f64 = 5.0;
        let expected_dist_so2 = PI;
        let expected_total_dist =
            (expected_dist_r2.powi(2) + (0.5 * expected_dist_so2).powi(2)).sqrt();

        assert!((space.distance(&state1, &state2) - expected_total_dist).abs() < 1e-9);
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
