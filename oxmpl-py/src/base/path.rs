// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::{prelude::*, types::PyList};
use std::sync::Arc;

use super::real_vector_state::PyRealVectorState;
use super::so2_state::PySO2State;
use oxmpl::base::{
    planner::Path as OxmplPath,
    state::{RealVectorState as OxmplRealVectorState, SO2State as OxmplSO2State},
};

#[derive(Clone)]
pub enum PathVariant {
    RealVector(OxmplPath<OxmplRealVectorState>),
    SO2(OxmplPath<OxmplSO2State>),
}

/// A sequence of states representing a solution path found by a planner.
#[pyclass(name = "Path", unsendable)]
#[derive(Clone)]
pub struct PyPath(pub PathVariant);

#[pymethods]
impl PyPath {
    /// Creates a new Path object from a list of states.
    #[staticmethod]
    fn from_real_vector_states(states: Vec<PyRealVectorState>) -> Self {
        let rust_states = states.into_iter().map(|s| (*s.0).clone()).collect();
        Self(PathVariant::RealVector(OxmplPath(rust_states)))
    }

    /// Creates a new Path from a list of `SO2State` objects.
    #[staticmethod]
    fn from_so2_states(states: Vec<PySO2State>) -> Self {
        let rust_states = states.into_iter().map(|s| (*s.0).clone()).collect();
        Self(PathVariant::SO2(OxmplPath(rust_states)))
    }

    /// list[RealVectorState]: The sequence of states that make up the path.
    #[getter]
    fn get_states(&self, py: Python<'_>) -> PyResult<PyObject> {
        let py_list = match &self.0 {
            PathVariant::RealVector(path) => {
                let list = PyList::empty(py);
                for s in &path.0 {
                    let py_state = PyRealVectorState(Arc::new(s.clone()));
                    let obj = py_state.into_pyobject(py)?; // Bound<PyAny>
                    list.append(obj)?;
                }
                list
            }
            PathVariant::SO2(path) => {
                let list = PyList::empty(py);
                for s in &path.0 {
                    let py_state = PySO2State(Arc::new(s.clone()));
                    let obj = py_state.into_pyobject(py)?;
                    list.append(obj)?;
                }
                list
            }
        };
        Ok(py_list.into())
    }

    /// The number of states in the path.
    fn __len__(&self) -> usize {
        match &self.0 {
            PathVariant::RealVector(path) => path.0.len(),
            PathVariant::SO2(path) => path.0.len(),
        }
    }

    fn __repr__(&self) -> String {
        let (len, type_name) = match &self.0 {
            PathVariant::RealVector(path) => (path.0.len(), "RealVectorState"),
            PathVariant::SO2(path) => (path.0.len(), "SO2State"),
        };
        format!("<Path of {len} {type_name}s>")
    }
}

impl From<OxmplPath<OxmplRealVectorState>> for PyPath {
    fn from(path: OxmplPath<OxmplRealVectorState>) -> Self {
        Self(PathVariant::RealVector(path))
    }
}

impl From<OxmplPath<OxmplSO2State>> for PyPath {
    fn from(path: OxmplPath<OxmplSO2State>) -> Self {
        Self(PathVariant::SO2(path))
    }
}
