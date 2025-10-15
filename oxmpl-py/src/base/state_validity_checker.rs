// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::prelude::*;
use std::rc::Rc;
use std::sync::Arc;

use oxmpl::base::{
    state::{
        CompoundState as OxmplCompoundState, RealVectorState as OxmplRealVectorState,
        SE2State as OxmplSE2State, SE3State as OxmplSE3State, SO2State as OxmplSO2State,
        SO3State as OxmplSO3State,
    },
    validity::StateValidityChecker,
};

use super::{
    compound_state::PyCompoundState, real_vector_state::PyRealVectorState, se2_state::PySE2State,
    se3_state::PySE3State, so2_state::PySO2State, so3_state::PySO3State,
};

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

impl StateValidityChecker<OxmplSO3State> for PyStateValidityChecker {
    fn is_valid(&self, state: &OxmplSO3State) -> bool {
        Python::with_gil(|py| {
            let result: PyResult<bool> = (move || {
                let py_state = Py::new(py, PySO3State(Arc::new(state.clone())))?;
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

impl StateValidityChecker<OxmplCompoundState> for PyStateValidityChecker {
    fn is_valid(&self, state: &OxmplCompoundState) -> bool {
        Python::with_gil(|py| {
            let result: PyResult<bool> = (move || {
                let py_state = Py::new(py, PyCompoundState(Rc::new(state.clone())))?;
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

impl StateValidityChecker<OxmplSE2State> for PyStateValidityChecker {
    fn is_valid(&self, state: &OxmplSE2State) -> bool {
        Python::with_gil(|py| {
            let result: PyResult<bool> = (move || {
                let py_state = Py::new(py, PySE2State(Rc::new(state.clone())))?;
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

impl StateValidityChecker<OxmplSE3State> for PyStateValidityChecker {
    fn is_valid(&self, state: &OxmplSE3State) -> bool {
        Python::with_gil(|py| {
            let result: PyResult<bool> = (move || {
                let py_state = Py::new(py, PySE3State(Rc::new(state.clone())))?;
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
