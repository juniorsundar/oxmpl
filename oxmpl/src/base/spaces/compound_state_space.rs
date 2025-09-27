// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use rand::Rng;

use crate::base::{
    error::StateSamplingError,
    space::{AnyStateSpace, StateSpace},
    state::CompoundState,
};

/// A state space that is composed of multiple other state spaces.
///
/// This is a key component for representing complex configuration spaces, such as the SE(2) space
/// for a 2D robot (combining 2D position and 1D rotation) or the configuration space of a
/// multi-jointed robot arm.
///
/// # Examples
///
/// Creating an SE(2) space for a 2D robot:
/// ```
/// use std::sync::Arc;
/// use oxmpl::base::space::{CompoundStateSpace, RealVectorStateSpace, SO2StateSpace, StateSpace};
/// use oxmpl::base::state::{CompoundState, RealVectorState, SO2State};
///
/// // 1. Create the component spaces.
/// let rvs_space = RealVectorStateSpace::new(2, Some(vec![(-1.0, 1.0), (-1.0, 1.0)])).unwrap();
/// let so2_space = SO2StateSpace::new(None).unwrap();
///
/// // 2. Create the CompoundStateSpace, giving equal weight to position and rotation.
/// let se2_space = CompoundStateSpace::new(
///     vec![
///         Box::new(rvs_space),
///         Box::new(so2_space)
///     ],
///     vec![1.0, 1.0], // weights
/// );
///
/// // 3. Create a compound state for this space.
/// let state = CompoundState {
///     components: vec![
///         Box::new(RealVectorState::new(vec![0.5, 0.5])),
///         Box::new(SO2State::new(std::f64::consts::PI / 2.0)),
///     ],
/// };
///
/// assert!(se2_space.satisfies_bounds(&state));
/// ```
pub struct CompoundStateSpace {
    /// The component state spaces, as Box-dyn StateSpaces.
    pub subspaces: Vec<Box<dyn AnyStateSpace>>,
    /// The weight of each component's contribution to the total distance.
    pub weights: Vec<f64>,
}

impl CompoundStateSpace {
    /// Creates a new `CompoundStateSpace`.
    ///
    /// # Panics
    ///
    /// Panics if the number of subspaces does not match the number of weights.
    pub fn new(subspaces: Vec<Box<dyn AnyStateSpace>>, weights: Vec<f64>) -> Self {
        assert_eq!(
            subspaces.len(),
            weights.len(),
            "Number of subspaces must match number of weights."
        );
        Self { subspaces, weights }
    }
}

impl StateSpace for CompoundStateSpace {
    type StateType = CompoundState;

    /// Computes the weighted Euclidean distance between two compound states.
    ///
    /// The total distance is `sqrt(sum((weight_i * dist_i)^2))`.
    fn distance(&self, state1: &Self::StateType, state2: &Self::StateType) -> f64 {
        let mut total_dist_sq = 0.0;
        for i in 0..self.subspaces.len() {
            let component_dist =
                self.subspaces[i].distance_dyn(&*state1.components[i], &*state2.components[i]);
            total_dist_sq += (component_dist * self.weights[i]).powi(2);
        }
        total_dist_sq.sqrt()
    }

    /// Interpolates between two compound states by interpolating each component.
    fn interpolate(
        &self,
        from: &Self::StateType,
        to: &Self::StateType,
        t: f64,
        out_state: &mut Self::StateType,
    ) {
        for i in 0..self.subspaces.len() {
            self.subspaces[i].interpolate_dyn(
                &*from.components[i],
                &*to.components[i],
                t,
                &mut *out_state.components[i],
            );
        }
    }

    /// Generates a random state by sampling from each subspace and combining the results.
    fn sample_uniform(&self, rng: &mut impl Rng) -> Result<Self::StateType, StateSamplingError> {
        let mut components = Vec::with_capacity(self.subspaces.len());
        for subspace in &self.subspaces {
            let component_state = subspace.sample_uniform_dyn(rng)?;
            components.push(component_state);
        }
        Ok(CompoundState { components })
    }

    /// Enforces the bounds of a compound state by enforcing the bounds on each component.
    fn enforce_bounds(&self, state: &mut Self::StateType) {
        for i in 0..self.subspaces.len() {
            self.subspaces[i].enforce_bounds_dyn(&mut *state.components[i]);
        }
    }

    /// Checks if a compound state is valid by checking if all its components are valid.
    /// Returns `true` only if all components satisfy their respective subspace bounds.
    fn satisfies_bounds(&self, state: &Self::StateType) -> bool {
        for i in 0..self.subspaces.len() {
            if !self.subspaces[i].satisfies_bounds_dyn(&*state.components[i]) {
                return false;
            }
        }
        true
    }

    /// Calculates the weighted root mean square of the subspaces' longest valid segment lengths.
    fn get_longest_valid_segment_length(&self) -> f64 {
        let mut total_longest_valid_segment_length_sq = 0.0;
        for i in 0..self.subspaces.len() {
            let component_longest_valid_segment_length =
                self.subspaces[i].get_longest_valid_segment_length_dyn();
            total_longest_valid_segment_length_sq +=
                (component_longest_valid_segment_length * self.weights[i]).powi(2);
        }
        total_longest_valid_segment_length_sq.sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::base::{
        space::{RealVectorStateSpace, SO2StateSpace},
        state::{RealVectorState, SO2State},
    };
    use std::f64::consts::PI;

    #[test]
    fn test_single_subspace() {
        let rvs = RealVectorStateSpace::new(2, Some(vec![(-5.0, 5.0), (-5.0, 5.0)])).unwrap();
        let space = CompoundStateSpace::new(vec![Box::new(rvs)], vec![2.0]);

        let state1 = CompoundState {
            components: vec![Box::new(RealVectorState::new(vec![0.0, 0.0]))],
        };
        let state2 = CompoundState {
            components: vec![Box::new(RealVectorState::new(vec![3.0, 4.0]))],
        };

        assert!((space.distance(&state1, &state2) - 10.0).abs() < 1e-9);
    }

    #[test]
    fn test_multiple_subspaces() {
        let rvs1 = RealVectorStateSpace::new(1, Some(vec![(-10.0, 10.0)])).unwrap();
        let so2 = SO2StateSpace::new(None).unwrap();
        let rvs2 = RealVectorStateSpace::new(1, Some(vec![(-10.0, 10.0)])).unwrap();

        let space = CompoundStateSpace::new(
            vec![Box::new(rvs1), Box::new(so2), Box::new(rvs2)],
            vec![1.0, 0.5, 1.0],
        );

        let state1 = CompoundState {
            components: vec![
                Box::new(RealVectorState::new(vec![1.0])),
                Box::new(SO2State::new(0.0)),
                Box::new(RealVectorState::new(vec![5.0])),
            ],
        };
        let state2 = CompoundState {
            components: vec![
                Box::new(RealVectorState::new(vec![2.0])),
                Box::new(SO2State::new(PI)),
                Box::new(RealVectorState::new(vec![1.0])),
            ],
        };

        let dist1_sq = (1.0f64 * 1.0).powi(2);
        let dist2_sq = (PI * 0.5).powi(2);
        let dist3_sq = (4.0f64 * 1.0).powi(2);
        let expected_dist = (dist1_sq + dist2_sq + dist3_sq).sqrt();

        assert!((space.distance(&state1, &state2) - expected_dist).abs() < 1e-9);

        let mut rng = rand::rng();
        let sample = space.sample_uniform(&mut rng);
        assert!(sample.is_ok());
        assert_eq!(sample.unwrap().components.len(), 3);
    }

    #[test]
    #[should_panic(expected = "Number of subspaces must match number of weights.")]
    fn test_mismatched_subspaces_and_weights() {
        let rvs = RealVectorStateSpace::new(2, Some(vec![(-1.0, 1.0), (-1.0, 1.0)])).unwrap();
        CompoundStateSpace::new(vec![Box::new(rvs)], vec![1.0, 1.0]);
    }
}
