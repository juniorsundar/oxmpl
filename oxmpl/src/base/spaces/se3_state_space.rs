// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::{
    error::StateSpaceError,
    space::{AnyStateSpace, CompoundStateSpace, RealVectorStateSpace, SO3StateSpace, StateSpace},
    state::SE3State,
};

/// A state space for 3D rigid body transformations (SE(3)).
///
/// This space combines a 3D translational space (`RealVectorStateSpace` of dimension 3) and a 3D
/// rotational space (`SO3StateSpace`). It allows for defining bounds for both translation (x, y,
/// z) and rotation (a maximum angle from a center rotation), and calculating weighted distances
/// between states.
#[derive(Clone)]
pub struct SE3StateSpace(pub CompoundStateSpace);

impl SE3StateSpace {
    /// Creates a new `SE3StateSpace`.
    ///
    /// The `translation_bounds` argument allows specifying the valid range for the translational
    /// part of the state. If provided, it must be a `Vec` corresponding to the bounds for x, y, and z.
    ///
    /// The `rotation_bounds` argument allows specifying the valid range for the rotational part of
    /// the state as a tuple `(center_rotation, max_angle)`.
    ///
    /// The `weight` parameter is applied to the rotational component when calculating distances,
    /// allowing control over the trade-off between translational and rotational costs.
    ///
    /// # Examples
    ///
    /// ```
    /// use oxmpl::base::space::SE3StateSpace;
    /// use oxmpl::base::state::SO3State;
    /// use std::f64::consts::PI;
    ///
    /// let space = SE3StateSpace::new(0.5, None).unwrap();
    ///
    /// let t_bounds = vec![(-1.0, 1.0), (-2.0, 2.0), (-3.0, 3.0)];
    /// let bounded_t_space = SE3StateSpace::new(1.0, Some(t_bounds)).unwrap();
    ///
    /// let t_bounds_2 = vec![(-1.0, 1.0), (-1.0, 1.0), (-1.0, 1.0)];
    /// let fully_bounded_space = SE3StateSpace::new(1.0, Some(t_bounds_2)).unwrap();
    /// ```
    pub fn new(
        weight: f64,
        bounds_option: Option<Vec<(f64, f64)>>,
    ) -> Result<Self, StateSpaceError> {
        let (r3, so3) = match bounds_option {
            Some(bounds) => {
                if bounds.len() != 3 {
                    return Err(StateSpaceError::DimensionMismatch {
                        expected: 3,
                        found: bounds.len(),
                    });
                } else {
                    (
                        RealVectorStateSpace::new(3, Some(vec![bounds[0], bounds[1], bounds[2]]))?,
                        SO3StateSpace::new(None)?,
                    )
                }
            }
            None => (
                RealVectorStateSpace::new(3, None)?,
                SO3StateSpace::new(None)?,
            ),
        };

        let compound_space =
            CompoundStateSpace::new(vec![Box::new(r3), Box::new(so3)], vec![1.0, weight]);
        Ok(SE3StateSpace(compound_space))
    }
}

impl StateSpace for SE3StateSpace {
    type StateType = SE3State;

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
        Ok(SE3State(compound_state))
    }

    fn get_longest_valid_segment_length(&self) -> f64 {
        self.0.get_longest_valid_segment_length_dyn()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::base::state::SO3State;
    use rand::rng;
    use std::f64::consts::PI;

    #[test]
    fn test_se3_space_creation() {
        assert!(SE3StateSpace::new(0.5, None).is_ok());

        let bounds = vec![(-1.0, 1.0), (-2.0, 2.0), (-3.0, 3.0)];
        assert!(SE3StateSpace::new(1.0, Some(bounds)).is_ok());
    }

    #[test]
    fn test_se3_space_creation_invalid_bounds() {
        let bounds = vec![(-1.0, 1.0)];
        let result = SE3StateSpace::new(1.0, Some(bounds));
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
        let space = SE3StateSpace::new(0.5, None).unwrap();
        let rot1 = SO3State::identity();
        let rot2 = SO3State::new(0.0, 0.0, 1.0, 0.0);
        let state1 = SE3State::new(0.0, 0.0, 0.0, rot1);
        let state2 = SE3State::new(3.0, 4.0, 0.0, rot2);

        let expected_dist_r3: f64 = 5.0;
        let expected_dist_so3 = PI;
        let expected_total_dist =
            (expected_dist_r3.powi(2) + (0.5 * expected_dist_so3).powi(2)).sqrt();

        assert!((space.distance(&state1, &state2) - expected_total_dist).abs() < 1e-9);
    }

    #[test]
    fn test_interpolate() {
        let space = SE3StateSpace::new(1.0, None).unwrap();
        let rot1 = SO3State::identity();

        let rot2 = SO3State::new(1.0 / 2.0_f64.sqrt(), 0.0, 0.0, 1.0 / 2.0_f64.sqrt());
        let state1 = SE3State::new(0.0, 0.0, 0.0, rot1);
        let state2 = SE3State::new(10.0, -10.0, 20.0, rot2);
        let mut interpolated_state = SE3State::new(0.0, 0.0, 0.0, SO3State::identity());

        space.interpolate(&state1, &state2, 0.5, &mut interpolated_state);

        assert_eq!(interpolated_state.get_x(), 5.0);
        assert_eq!(interpolated_state.get_y(), -5.0);
        assert_eq!(interpolated_state.get_z(), 10.0);

        let dist_to_mid = space.distance(&state1, &interpolated_state);
        let total_dist = space.distance(&state1, &state2);
        assert!((dist_to_mid - total_dist / 2.0).abs() < 1e-9);
    }

    #[test]
    fn test_bounds() {
        let bounds = vec![(-1.0, 1.0), (-2.0, 2.0), (-3.0, 3.0)];
        let space = SE3StateSpace::new(1.0, Some(bounds)).unwrap();

        let mut out_of_bounds_state = SE3State::new(2.0, -3.0, 4.0, SO3State::identity());
        let in_bounds_state = SE3State::new(0.5, 1.5, -2.5, SO3State::identity());

        assert!(!space.satisfies_bounds(&out_of_bounds_state));
        assert!(space.satisfies_bounds(&in_bounds_state));

        space.enforce_bounds(&mut out_of_bounds_state);
        assert_eq!(out_of_bounds_state.get_x(), 1.0);
        assert_eq!(out_of_bounds_state.get_y(), -2.0);
        assert_eq!(out_of_bounds_state.get_z(), 3.0);
    }

    #[test]
    fn test_sample_uniform() {
        let bounds = vec![(-1.0, 1.0), (5.0, 10.0), (0.0, 2.0)];
        let space = SE3StateSpace::new(1.0, Some(bounds)).unwrap();
        let mut rng = rng();

        for _ in 0..100 {
            let sample = space.sample_uniform(&mut rng).unwrap();
            assert!(space.satisfies_bounds(&sample));
        }
    }
}
