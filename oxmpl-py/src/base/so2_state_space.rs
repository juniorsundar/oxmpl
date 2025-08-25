// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::{exceptions::PyValueError, prelude::*};
use std::sync::{Arc, Mutex};

use oxmpl::base::space::{SO2StateSpace as OxmplSO2StateSpace, StateSpace as _};

use super::so2_state::PySO2State;

/// Defines an N-dimensional space for `SO2State` instances.
///
/// This class defines the planning space, including its dimensionality and boundaries.
#[pyclass(name = "SO2StateSpace", unsendable)]
#[derive(Clone)]
pub struct PySO2StateSpace(pub Arc<Mutex<OxmplSO2StateSpace>>);

#[pymethods]
impl PySO2StateSpace {
    /// Creates a new `SO2StateSpace`.
    ///
    /// Args:
    ///     bounds (Optional[[Tuple[float, float]]): If provided, defines the
    ///         min and max. If `None`, the space is bounded in [-PI, PI).
    ///
    /// Raises:
    ///     ValueError: If the provided inputs are invalid.
    #[new]
    #[pyo3(signature = (bounds=None))]
    fn new(bounds: Option<(f64, f64)>) -> PyResult<Self> {
        match OxmplSO2StateSpace::new(bounds) {
            Ok(space) => Ok(Self(Arc::new(Mutex::new(space)))),
            Err(e) => Err(PyValueError::new_err(e.to_string())),
        }
    }

    /// Computes the Euclidean (L2) distance between two states.
    fn distance(&self, state1: &PySO2State, state2: &PySO2State) -> f64 {
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
