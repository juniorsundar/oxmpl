// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::prelude::*;
use rand::Rng;
use std::sync::Arc;

use oxmpl::base::{
    error::StateSamplingError,
    goal::{Goal, GoalRegion, GoalSampleableRegion},
    state::RealVectorState as OxmplRealVectorState,
};

use super::real_vector_state::PyRealVectorState;

/// An internal Rust struct that implements the `Goal` trait hierarchy
/// by calling methods on a user-provided Python goal object.
pub struct PyGoal {
    pub instance: PyObject,
}
impl Clone for PyGoal {
    fn clone(&self) -> Self {
        Python::with_gil(|py| Self {
            instance: self.instance.clone_ref(py),
        })
    }
}

impl Goal<OxmplRealVectorState> for PyGoal {
    fn is_satisfied(&self, state: &OxmplRealVectorState) -> bool {
        Python::with_gil(|py| {
            let py_state = PyRealVectorState(Arc::new(state.clone()));
            self.instance
                .call_method1(py, "is_satisfied", (py_state,))
                .and_then(|res| res.extract(py))
                .unwrap_or(false)
        })
    }
}
impl GoalRegion<OxmplRealVectorState> for PyGoal {
    fn distance_goal(&self, state: &OxmplRealVectorState) -> f64 {
        Python::with_gil(|py| {
            let py_state = PyRealVectorState(Arc::new(state.clone()));
            self.instance
                .call_method1(py, "distance_goal", (py_state,))
                .and_then(|res| res.extract(py))
                .unwrap_or(f64::INFINITY)
        })
    }
}
impl GoalSampleableRegion<OxmplRealVectorState> for PyGoal {
    fn sample_goal(&self, _rng: &mut impl Rng) -> Result<OxmplRealVectorState, StateSamplingError> {
        Python::with_gil(|py| {
            self.instance
                .call_method0(py, "sample_goal")
                .and_then(|res| res.extract::<PyRealVectorState>(py))
                .map(|py_state| (*py_state.0).clone())
                .map_err(|e| {
                    e.print(py);
                    StateSamplingError::GoalRegionUnsatisfiable
                })
        })
    }
}

