// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::{exceptions::PyValueError, prelude::*};
use std::sync::{Arc, Mutex};

use oxmpl::base::space::{RealVectorStateSpace as OxmplRealVectorStateSpace, StateSpace as _};

use super::real_vector_state::PyRealVectorState;

/// Defines an N-dimensional space for `RealVectorState` instances.
///
/// This class defines the planning space, including its dimensionality and boundaries.
#[pyclass(name = "RealVectorStateSpace", unsendable)]
#[derive(Clone)]
pub struct PyRealVectorStateSpace(pub Arc<Mutex<OxmplRealVectorStateSpace>>);

#[pymethods]
impl PyRealVectorStateSpace {
    /// Creates a new `RealVectorStateSpace`.
    ///
    /// Args:
    ///     dimension (int): The number of dimensions for the space.
    ///     bounds (Optional[List[Tuple[float, float]]]): If provided, defines the
    ///         min and max for each dimension. If `None`, the space is unbounded.
    ///
    /// Raises:
    ///     ValueError: If the provided inputs are invalid.
    #[new]
    #[pyo3(signature = (dimension, bounds=None))]
    fn new(dimension: usize, bounds: Option<Vec<(f64, f64)>>) -> PyResult<Self> {
        match OxmplRealVectorStateSpace::new(dimension, bounds) {
            Ok(space) => Ok(Self(Arc::new(Mutex::new(space)))),
            Err(e) => Err(PyValueError::new_err(e.to_string())),
        }
    }

    /// Computes the Euclidean (L2) distance between two states.
    fn distance(&self, state1: &PyRealVectorState, state2: &PyRealVectorState) -> f64 {
        // This calls the real Rust implementation.
        self.0.lock().unwrap().distance(&state1.0, &state2.0)
    }

    /// Returns the maximum possible distance in this space.
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
