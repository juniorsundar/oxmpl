// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::prelude::*;
use std::sync::Arc;

use oxmpl::base::{planner::Path as OxmplPath, state::RealVectorState as OxmplRealVectorState};

use super::real_vector_state::PyRealVectorState;

/// A sequence of states representing a solution path found by a planner.
#[pyclass(name = "Path")]
#[derive(Clone)]
pub struct PyPath(pub OxmplPath<OxmplRealVectorState>);

#[pymethods]
impl PyPath {
    /// Creates a new Path object from a list of states.
    #[new]
    fn py_new(states: Vec<PyRealVectorState>) -> Self {
        let rust_states = states.into_iter().map(|s| (*s.0).clone()).collect();
        Self(OxmplPath(rust_states))
    }

    /// list[RealVectorState]: The sequence of states that make up the path.
    #[getter]
    fn get_states(&self) -> Vec<PyRealVectorState> {
        self.0
             .0
            .iter()
            .map(|s| PyRealVectorState(Arc::new(s.clone())))
            .collect()
    }

    /// The number of states in the path.
    fn __len__(&self) -> usize {
        self.0 .0.len()
    }

    fn __repr__(&self) -> String {
        format!("<Path with {} states>", self.0 .0.len())
    }
}

impl PyPath {
    /// An internal helper to create a `PyPath` from a core Rust path. Not exposed to Python.
    pub fn from_rust_path(path: OxmplPath<OxmplRealVectorState>) -> Self {
        Self(path)
    }
}


