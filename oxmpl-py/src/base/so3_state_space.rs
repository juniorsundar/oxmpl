// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::{exceptions::PyValueError, prelude::*};
use std::sync::{Arc, Mutex};

use oxmpl::base::{
    space::{SO3StateSpace as OxmplSO3StateSpace, StateSpace as _},
    state::SO3State as OxmplSO3State,
};

use crate::base::py_state_convert::PyStateConvert;

use super::so3_state::PySO3State;

/// Defines an N-dimensional space for `SO3State` instances.
///
/// This class defines the planning space, including its dimensionality and boundaries.
#[pyclass(name = "SO3StateSpace", unsendable)]
#[derive(Clone)]
pub struct PySO3StateSpace(pub Arc<Mutex<OxmplSO3StateSpace>>);

#[pymethods]
impl PySO3StateSpace {
    /// Creates a new `SO3StateSpace`.
    ///
    /// Args:
    ///     bounds (Optional[[Tuple[float, float]]): If provided, defines the
    ///         min and max. If `None`, the space is bounded in [-PI, PI).
    ///
    /// Raises:
    ///     ValueError: If the provided inputs are invalid.
    #[new]
    #[pyo3(signature = (bounds=None))]
    fn new(bounds: Option<(PySO3State, f64)>) -> PyResult<Self> {
        let re_bounds = bounds.map(|(center_rotation, max_angle)| {
            (OxmplSO3State::from_py_wrapper(center_rotation), max_angle)
        });
        match OxmplSO3StateSpace::new(re_bounds) {
            Ok(space) => Ok(Self(Arc::new(Mutex::new(space)))),
            Err(e) => Err(PyValueError::new_err(e.to_string())),
        }
    }

    /// Computes the Euclidean (L2) distance between two states.
    fn distance(&self, state1: &PySO3State, state2: &PySO3State) -> f64 {
        // This calls the real Rust implementation.
        self.0.lock().unwrap().distance(&state1.0, &state2.0)
    }

    /// Returns the maximum possible distance in this space, which is always PI.
    fn get_maximum_extent(&self) -> f64 {
        self.0.lock().unwrap().get_maximum_extent()
    }

    /// Sets the fraction used to determine motion checking resolution.
    fn set_longest_valid_segment_fraction(&mut self, fraction: f64) {
        self.0
            .lock()
            .unwrap()
            .set_longest_valid_segment_fraction(fraction);
    }
}
