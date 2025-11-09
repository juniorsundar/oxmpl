// Copyright (c) 2025 Ross Gardiner, Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::{goal::JsGoal, real_vector_state_space::JsRealVectorStateSpace};
use oxmpl::base::{
    problem_definition::ProblemDefinition, space::RealVectorStateSpace, state::RealVectorState,
};
use std::sync::Arc;
use wasm_bindgen::prelude::*;

#[wasm_bindgen(js_name = ProblemDefinition)]
pub struct JsProblemDefinition {
    inner: Arc<ProblemDefinition<RealVectorState, RealVectorStateSpace, JsGoal>>,
}

#[wasm_bindgen(js_class = ProblemDefinition)]
impl JsProblemDefinition {
    #[wasm_bindgen(constructor)]
    pub fn new(space: &JsRealVectorStateSpace, start: Vec<f64>, goal: JsGoal) -> Self {
        let start_state = RealVectorState::new(start);
        let problem_def = ProblemDefinition {
            space: space.inner.clone(),
            start_states: vec![start_state],
            goal: Arc::new(goal),
        };
        Self {
            inner: Arc::new(problem_def),
        }
    }

    #[wasm_bindgen(js_name = getStart)]
    pub fn get_start(&self) -> Vec<f64> {
        self.inner.start_states[0].values.clone()
    }

    #[wasm_bindgen(js_name = getDimension)]
    pub fn get_dimension(&self) -> usize {
        self.inner.space.dimension
    }
}

impl From<&JsProblemDefinition>
    for ProblemDefinition<RealVectorState, RealVectorStateSpace, JsGoal>
{
    fn from(js_problem: &JsProblemDefinition) -> Self {
        ProblemDefinition {
            space: js_problem.inner.space.clone(),
            start_states: js_problem.inner.start_states.clone(),
            goal: js_problem.inner.goal.clone(),
        }
    }
}
