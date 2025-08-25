// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::prelude::*;
use std::sync::Arc;

use oxmpl::base::state::RealVectorState as OxmplRealVectorState;

/// A state in an N-dimensional Euclidean space (R^n).
///
/// This class represents a single point or configuration, defined by a list of floating-point
/// numbers.
///
/// Args:
///     values (List[float]): A list of numbers representing the state's components.
#[pyclass(name = "RealVectorState", unsendable)]
#[derive(Clone)]
pub struct PyRealVectorState(pub Arc<OxmplRealVectorState>);

#[pymethods]
impl PyRealVectorState {
    #[new]
    fn new(values: Vec<f64>) -> Self {
        // Creates the underlying Rust struct and wraps it for Python.
        let state = OxmplRealVectorState::new(values);
        Self(Arc::new(state))
    }

    /// list[float]: The components of the state vector.
    #[getter]
    fn get_values(&self) -> Vec<f64> {
        self.0.values.clone()
    }

    fn __repr__(&self) -> String {
        format!("<RealVectorState values={:?}>", self.0.values)
    }
}
