// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::prelude::*;
use std::sync::Arc;

use oxmpl::base::{
    state::RealVectorState as OxmplRealVectorState, state::SO2State as OxmplSO2State,
    validity::StateValidityChecker,
};

use super::real_vector_state::PyRealVectorState;
use super::so2_state::PySO2State;

/// An internal Rust struct that implements the `StateValidityChecker` trait by calling a
/// user-provided Python function.
pub struct PyStateValidityChecker {
    pub callback: PyObject,
}
impl Clone for PyStateValidityChecker {
    fn clone(&self) -> Self {
        Python::with_gil(|py| Self {
            callback: self.callback.clone_ref(py),
        })
    }
}

impl StateValidityChecker<OxmplRealVectorState> for PyStateValidityChecker {
    fn is_valid(&self, state: &OxmplRealVectorState) -> bool {
        Python::with_gil(|py| {
            let result: PyResult<bool> = (move || {
                let py_state = Py::new(py, PyRealVectorState(Arc::new(state.clone())))?;
                let args = (py_state,);
                let result = self.callback.call1(py, args)?;
                result.extract(py)
            })();
            match result {
                Ok(is_valid) => is_valid,
                Err(e) => {
                    e.print(py);
                    false
                }
            }
        })
    }
}

impl StateValidityChecker<OxmplSO2State> for PyStateValidityChecker {
    fn is_valid(&self, state: &OxmplSO2State) -> bool {
        Python::with_gil(|py| {
            let result: PyResult<bool> = (move || {
                let py_state = Py::new(py, PySO2State(Arc::new(state.clone())))?;
                let args = (py_state,);
                let result = self.callback.call1(py, args)?;
                result.extract(py)
            })();
            match result {
                Ok(is_valid) => is_valid,
                Err(e) => {
                    e.print(py);
                    false
                }
            }
        })
    }
}
