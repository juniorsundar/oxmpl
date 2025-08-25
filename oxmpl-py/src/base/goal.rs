// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::prelude::*;
use rand::Rng;
use std::marker::PhantomData;

use oxmpl::base::{
    error::StateSamplingError,
    goal::{Goal, GoalRegion, GoalSampleableRegion},
    state,
};

use super::py_state_convert::PyStateConvert;

pub struct PyGoal<State> {
    pub instance: PyObject,
    pub _phantom: PhantomData<State>,
}

impl<State> Clone for PyGoal<State> {
    fn clone(&self) -> Self {
        Python::with_gil(|py| Self {
            instance: self.instance.clone_ref(py),
            _phantom: PhantomData,
        })
    }
}

// Implement the Goal traits for ANY state type that satisfies our conversion trait.
impl<State: PyStateConvert + state::State> Goal<State> for PyGoal<State> {
    fn is_satisfied(&self, state: &State) -> bool {
        Python::with_gil(|py| {
            let py_state = state.to_py_wrapper();
            self.instance
                .call_method1(py, "is_satisfied", (py_state,))
                .and_then(|res| res.extract(py))
                .unwrap_or(false)
        })
    }
}

impl<State: PyStateConvert + state::State> GoalRegion<State> for PyGoal<State> {
    fn distance_goal(&self, state: &State) -> f64 {
        Python::with_gil(|py| {
            let py_state = state.to_py_wrapper();
            self.instance
                .call_method1(py, "distance_goal", (py_state,))
                .and_then(|res| res.extract(py))
                .unwrap_or(f64::INFINITY)
        })
    }
}

impl<State: PyStateConvert + state::State> GoalSampleableRegion<State> for PyGoal<State> {
    fn sample_goal(&self, _rng: &mut impl Rng) -> Result<State, StateSamplingError> {
        Python::with_gil(|py| {
            self.instance
                .call_method0(py, "sample_goal")
                .and_then(|res| res.extract::<State::Wrapper>(py))
                .map(State::from_py_wrapper)
                .map_err(|e| {
                    e.print(py);
                    StateSamplingError::GoalRegionUnsatisfiable
                })
        })
    }
}
