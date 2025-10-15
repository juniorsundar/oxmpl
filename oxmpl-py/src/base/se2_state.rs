// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::prelude::*;
use std::{rc::Rc, sync::Arc};

use oxmpl::base::state::SE2State as OxmplSE2State;

use crate::base::{PyRealVectorState, PySO2State};

/// A state representing a 2D rigid body transformation (x, y, yaw).
///
/// This is an element of the Special Euclidean group SE(2), combining a 2D translation and a 2D
/// rotation.
///
/// Args:
///     x (float): The x-coordinate of the translation.
///     y (float): The y-coordinate of the translation.
///     yaw (float): The rotation angle in radians.
#[pyclass(name = "SE2State", unsendable)]
#[derive(Clone)]
pub struct PySE2State(pub Rc<OxmplSE2State>);

#[pymethods]
impl PySE2State {
    #[new]
    fn new(x: f64, y: f64, yaw: f64) -> Self {
        let state = OxmplSE2State::new(x, y, yaw);
        Self(Rc::new(state))
    }

    /// float: The x-coordinate of the state.
    #[getter]
    fn get_x(&self) -> f64 {
        self.0.get_x()
    }

    /// float: The y-coordinate of the state.
    #[getter]
    fn get_y(&self) -> f64 {
        self.0.get_y()
    }

    /// float: The yaw (rotation) of the state in radians.
    #[getter]
    fn get_yaw(&self) -> f64 {
        self.0.get_yaw()
    }

    /// RealVectorState: The translational component (x, y) of the state.
    #[getter]
    fn get_translation(&self) -> PyRealVectorState {
        PyRealVectorState(Arc::new(self.0.get_translation().clone()))
    }

    /// SO2State: The rotational component (yaw) of the state.
    #[getter]
    fn get_rotation(&self) -> PySO2State {
        PySO2State(Arc::new(self.0.get_rotation().clone()))
    }

    fn __repr__(&self) -> String {
        format!(
            "<SE2State with x: {}, y: {}, yaw: {}>",
            self.0.get_x(),
            self.0.get_y(),
            self.0.get_yaw()
        )
    }
}
