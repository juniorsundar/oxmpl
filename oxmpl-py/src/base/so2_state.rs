// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::prelude::*;
use std::sync::Arc;

use oxmpl::base::state::SO2State as OxmplSO2State;

/// A state representing a 2D rotation, an element of the Special Orthogonal group SO(2).
///
/// The angle is stored in radians and is canonically represented in the range `[-PI, PI)`.
///
/// Args:
///     values (float): A number representing the state's components.
#[pyclass(name = "SO2State", unsendable)]
#[derive(Clone)]
pub struct PySO2State(pub Arc<OxmplSO2State>);

#[pymethods]
impl PySO2State {
    #[new]
    fn new(value: f64) -> Self {
        // Creates the underlying Rust struct and wraps it for Python.
        let state = OxmplSO2State::new(value);
        Self(Arc::new(state))
    }

    /// float: The component of the state.
    #[getter]
    fn get_value(&self) -> f64 {
        self.0.value
    }

    fn __repr__(&self) -> String {
        format!("<SO2State value={:?}>", self.0.value)
    }
}
