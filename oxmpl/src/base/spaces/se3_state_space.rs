// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::{
    error::StateSamplingError,
    error::StateSpaceError,
    space::{RealVectorStateSpace, SO3StateSpace, StateSpace},
    state::SE3State,
};

/// A state space for 3D rigid body transformations (SE(3)).
///
/// This space combines a 3D translational space (`RealVectorStateSpace` of dimension 3) and a 3D
/// rotational space (`SO3StateSpace`). It allows for defining bounds for both translation (x, y,
/// z) and rotation (a maximum angle from a center rotation), and calculating weighted distances
/// between states.
#[derive(Clone)]
pub struct SE3StateSpace {
    translation_space: RealVectorStateSpace,
    rotation_space: SO3StateSpace,
    weight: f64,
}

impl SE3StateSpace {
    /// Creates a new `SE3StateSpace`.
    ///
    /// The `bounds_option` argument allows specifying the valid range for the translational
    /// part of the state. If provided, it must be a `Vec` of exactly three `(min, max)` tuples
    /// corresponding to the bounds for x, y, and z. The rotational component (SO(3)) is always
    /// unbounded.
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

        Ok(SE3StateSpace {
            translation_space: r3,
            rotation_space: so3,
            weight,
        })
    }
}

impl StateSpace for SE3StateSpace {
    type StateType = SE3State;

    fn distance(&self, state1: &Self::StateType, state2: &Self::StateType) -> f64 {
        let trans1 = state1.get_translation();
        let trans2 = state2.get_translation();
        let rot1 = state1.get_rotation();
        let rot2 = state2.get_rotation();

        let dist_trans = self.translation_space.distance(trans1, trans2);
        let dist_rot = self.rotation_space.distance(rot1, rot2);

        (dist_trans.powi(2) + (self.weight * dist_rot).powi(2)).sqrt()
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
    ) -> Result<Self::StateType, StateSamplingError> {
        let trans = self.translation_space.sample_uniform(rng)?;
        let rot = self.rotation_space.sample_uniform(rng)?;
        Ok(SE3State::from_parts(trans, rot))
    }

    fn get_longest_valid_segment_length(&self) -> f64 {
        let trans_lvsl = self.translation_space.get_longest_valid_segment_length();
        let rot_lvsl = self.rotation_space.get_longest_valid_segment_length();
        (trans_lvsl.powi(2) + (self.weight * rot_lvsl).powi(2)).sqrt()
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
