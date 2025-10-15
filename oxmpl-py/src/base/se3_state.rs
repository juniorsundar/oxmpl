// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::prelude::*;
use std::{rc::Rc, sync::Arc};

use oxmpl::base::state::SE3State as OxmplSE3State;

use crate::base::{PyRealVectorState, PySO3State};

/// A state representing a 3D rigid body transformation (x, y, z, rotation).
///
/// This is an element of the Special Euclidean group SE(3), combining a 3D translation and a 3D
/// rotation (represented by a quaternion).
///
/// Args:
///     x (float): The x-coordinate of the translation.
///     y (float): The y-coordinate of the translation.
///     z (float): The z-coordinate of the translation.
///     rotation (SO3State): The rotational component of the state.
#[pyclass(name = "SE3State", unsendable)]
#[derive(Clone)]
pub struct PySE3State(pub Rc<OxmplSE3State>);

#[pymethods]
impl PySE3State {
    #[new]
    fn new(x: f64, y: f64, z: f64, rotation: PySO3State) -> Self {
        let state = OxmplSE3State::new(x, y, z, (*rotation.0).clone());
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

    /// float: The z-coordinate of the state.
    #[getter]
    fn get_z(&self) -> f64 {
        self.0.get_z()
    }

    /// RealVectorState: The translational component (x, y, z) of the state.
    #[getter]
    fn get_translation(&self) -> PyRealVectorState {
        PyRealVectorState(Arc::new(self.0.get_translation().clone()))
    }

    /// SO3State: The rotational component of the state.
    #[getter]
    fn get_rotation(&self) -> PySO3State {
        PySO3State(Arc::new(self.0.get_rotation().clone()))
    }

    fn __repr__(&self) -> String {
        format!(
            "<SE3State with x: {}, y: {}, z: {}, rotation: {}>",
            self.0.get_x(),
            self.0.get_y(),
            self.0.get_z(),
            self.0.get_rotation()
        )
    }
}
