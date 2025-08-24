// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use rand::Rng;

use crate::base::{
    error::{StateSamplingError, StateSpaceError},
    space::StateSpace,
    state::RealVectorState,
};

/// A state space representing an N-dimensional Euclidean space (R^n).
///
/// Most common state space for systems whose configuration can be described by a vector of real
/// numbers. Supports both bounded (hyper-rectangle) and unbounded configurations.
#[derive(Clone)]
pub struct RealVectorStateSpace {
    /// n-Dimensionality of VectorStateSpace i.e. R^n.
    pub dimension: usize,
    /// The bounds for each dimension, defining the valid region for planning. Each tuple is
    /// `(lower, upper)`. For unbounded dimensions it is`f64::NEG_INFINITY` and `f64::INFINITY`
    pub bounds: Vec<(f64, f64)>,

    longest_valid_segment_fraction: f64,
}

impl RealVectorStateSpace {
    /// Creates a new `RealVectorStateSpace`.
    ///
    /// This constructor allows for the creation of both bounded and unbounded spaces.
    /// To create a bounded space, provide a Some() vector of `(lower, upper)` tuples.
    /// To create an unbounded space, pass `None` for the bounds.
    ///
    /// # Arguments
    ///
    /// * `dimension` - The number of dimensions for the space.
    /// * `bounds_option` - An optional vector of `(min, max)` tuples.
    ///   - If `Some(bounds)`, the length of `bounds` must match `dimension`.
    ///   - If `None`, the space is initialized as unbounded in all dimensions.
    ///
    /// # Errors
    ///
    /// This function will return an error if:
    /// * `StateSpaceError::DimensionMismatch`: The provided bounds vector has a different
    ///   length than the specified `dimension`.
    /// * `StateSpaceError::InvalidBound`: A lower bound is greater than or equal to its
    ///   corresponding upper bound.
    /// * `StateSpaceError::ZeroDimensionUnbounded`: An attempt is made to create an unbounded
    ///   space with zero dimensions.
    ///
    /// # Examples
    ///
    /// ```
    /// use oxmpl::base::space::RealVectorStateSpace;
    ///
    /// let bounds = vec![(-1.0, 1.0), (-2.0, 2.0)];
    /// let bounded_space = RealVectorStateSpace::new(2, Some(bounds)).unwrap();
    /// assert_eq!(bounded_space.dimension, 2);
    ///
    /// let unbounded_space = RealVectorStateSpace::new(3, None).unwrap();
    /// assert_eq!(unbounded_space.dimension, 3);
    /// assert_eq!(unbounded_space.bounds[0], (f64::NEG_INFINITY, f64::INFINITY));
    /// ```
    pub fn new(
        dimension: usize,
        bounds_option: Option<Vec<(f64, f64)>>,
    ) -> Result<Self, StateSpaceError> {
        let bounds = match bounds_option {
            Some(explicit_bounds) => {
                if explicit_bounds.len() != dimension {
                    return Err(StateSpaceError::DimensionMismatch {
                        expected: dimension,
                        found: explicit_bounds.len(),
                    });
                }
                for bound in &explicit_bounds {
                    if bound.0 >= bound.1 {
                        return Err(StateSpaceError::InvalidBound {
                            lower: bound.0,
                            upper: bound.1,
                        });
                    }
                }
                explicit_bounds
            }
            None => {
                if dimension == 0 {
                    return Err(StateSpaceError::ZeroDimensionUnbounded);
                }
                vec![(f64::NEG_INFINITY, f64::INFINITY); dimension]
            }
        };

        Ok(Self {
            dimension,
            bounds,
            longest_valid_segment_fraction: 0.05,
        })
    }

    /// A helper to calculate the diagonal of the space's bounding box.
    pub fn get_maximum_extent(&self) -> f64 {
        if self
            .bounds
            .iter()
            .any(|(low, high)| !low.is_finite() || !high.is_finite())
        {
            1.0
        } else {
            let sum_sq_diff: f64 = self
                .bounds
                .iter()
                .map(|(low, high)| (high - low).powi(2))
                .sum();
            sum_sq_diff.sqrt()
        }
    }

    /// Allows a user to configure the motion checking resolution.
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

impl StateSpace for RealVectorStateSpace {
    type StateType = RealVectorState;

    /// Find distance between current state1 and target state2. Depends on StateSpace.
    /// In RealVectorStateSpace, this refers to the L2-norm.
    fn distance(&self, state1: &Self::StateType, state2: &Self::StateType) -> f64 {
        assert_eq!(
            state1.values.len(),
            self.dimension,
            "State1 has incorrect dimension for this space."
        );
        assert_eq!(
            state2.values.len(),
            self.dimension,
            "State2 has incorrect dimension for this space."
        );
        state1
            .values
            .iter()
            .zip(state2.values.iter())
            .map(|(v1, v2)| (v1 - v2).powi(2))
            .sum::<f64>()
            .sqrt()
    }

    /// Performs linear interpolation between two states.
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
        assert_eq!(
            from.values.len(),
            self.dimension,
            "From-state has incorrect dimension."
        );
        assert_eq!(
            to.values.len(),
            self.dimension,
            "To-state has incorrect dimension."
        );
        assert_eq!(
            out_state.values.len(),
            self.dimension,
            "Out-state has incorrect dimension or not properly initialized."
        );
        for i in 0..from.values.len() {
            out_state.values[i] = from.values[i] + (to.values[i] - from.values[i]) * t;
        }
    }

    /// Modifies the state by clamping each of its values to the space's bounds.
    fn enforce_bounds(&self, state: &mut Self::StateType) {
        if state.values.len() != self.dimension {
            assert_eq!(
                state.values.len(),
                self.dimension,
                "State and space dimension mismatch when enforcing bounds."
            );
        }
        for (i, value) in state.values.iter_mut().enumerate() {
            if i < self.bounds.len() {
                let (lower, upper) = self.bounds[i];
                *value = value.clamp(lower, upper);
            }
        }
    }

    /// Checks if a state is within the space's bounds, allowing for a small tolerance.
    ///
    /// This check uses a machine epsilon tolerance to prevent floating-point inaccuracies from
    /// incorrectly rejecting states that are numerically on the boundary.
    fn satisfies_bounds(&self, state: &Self::StateType) -> bool {
        if state.values.len() != self.dimension {
            assert_eq!(
                state.values.len(),
                self.dimension,
                "State and space dimension mismatch when checking bound satisfaction."
            );
        }
        for i in 0..self.dimension {
            let (lower, upper) = self.bounds[i];
            if state.values[i] - f64::EPSILON > upper || state.values[i] + f64::EPSILON < lower {
                return false;
            }
        }
        true
    }

    /// Generates a state uniformly at random from within the defined bounds.
    ///
    /// # Errors
    ///
    /// * `StateSamplingError::UnboundedDimension` if any dimension of the space is infinite.
    /// * `StateSamplingError::ZeroVolume` if any dimension has a lower bound greater than
    ///   or equal to its upper bound.
    fn sample_uniform(&self, rng: &mut impl Rng) -> Result<Self::StateType, StateSamplingError> {
        let mut values = Vec::with_capacity(self.dimension);

        for i in 0..self.dimension {
            let (lower, upper) = self.bounds[i];

            if !lower.is_finite() || !upper.is_finite() {
                return Err(StateSamplingError::UnboundedDimension { dimension_index: i });
            }
            if lower >= upper {
                return Err(StateSamplingError::ZeroVolume);
            }
            values.push(rng.random_range(lower..upper));
        }

        Ok(RealVectorState { values })
    }

    fn get_longest_valid_segment_length(&self) -> f64 {
        self.get_maximum_extent() * self.longest_valid_segment_fraction
    }
}
