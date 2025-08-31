// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::prelude::*;
use std::sync::Arc;

use oxmpl::base::state::SO3State as OxmplSO3State;

/// A state representing a 2D rotation, an element of the Special Orthogonal group SO(2).
///
/// The angle is stored in radians and is canonically represented in the range `[-PI, PI)`.
///
/// Args:
///     values (float): A number representing the state's components.
#[pyclass(name = "SO3State", unsendable)]
#[derive(Clone)]
pub struct PySO3State(pub Arc<OxmplSO3State>);

#[pymethods]
impl PySO3State {
    #[new]
    fn new(x: f64, y: f64, z: f64, w: f64) -> Self {
        // Creates the underlying Rust struct and wraps it for Python.
        let state = OxmplSO3State::new(x, y, z, w);
        Self(Arc::new(state))
    }

    /// float: x-component of Quaternion.
    #[getter]
    fn get_x(&self) -> f64 {
        self.0.x
    }
    /// float: y-component of Quaternion.
    #[getter]
    fn get_y(&self) -> f64 {
        self.0.y
    }
    /// float: z-component of Quaternion.
    #[getter]
    fn get_z(&self) -> f64 {
        self.0.z
    }
    /// float: w-component of Quaternion.
    #[getter]
    fn get_w(&self) -> f64 {
        self.0.w
    }

    #[staticmethod]
    fn identity() -> Self {
        Self(Arc::new(OxmplSO3State::new(0., 0., 0., 1.)))
    }

    fn __repr__(&self) -> String {
        format!(
            "<SO3State x={:?}, y={:?}, z={:?}, w={:?}>",
            self.0.x, self.0.y, self.0.z, self.0.w
        )
    }
}
