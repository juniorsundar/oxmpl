// Copyright (c) 2025 Ross Gardiner, Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::{
    goal::JsGoal, path::JsPath, planner::JsPlannerConfig, problem_definition::JsProblemDefinition,
    state_validity_checker::JsStateValidityChecker,
};
use oxmpl::base::{planner::Planner, space::RealVectorStateSpace, state::RealVectorState};
use oxmpl::geometric::RRTConnect;
use std::sync::Arc;
use std::time::Duration;
use wasm_bindgen::prelude::*;

#[wasm_bindgen(js_name = RRTConnect)]
pub struct JsRRTConnect {
    planner: RRTConnect<RealVectorState, RealVectorStateSpace, JsGoal>,
}

#[wasm_bindgen(js_class = RRTConnect)]
impl JsRRTConnect {
    /**
     * @param {number} max_distance
     * @param {number} goal_bias
     * @param {JsPlannerConfig} config
     */
    #[wasm_bindgen(constructor)]
    pub fn new(max_distance: f32, goal_bias: f32, config: &JsPlannerConfig) -> Self {
        Self {
            planner: RRTConnect::new(max_distance as f64, goal_bias as f64, &config.into()),
        }
    }

    pub fn setup(
        &mut self,
        problem_def: &JsProblemDefinition,
        validity_checker: &JsStateValidityChecker,
    ) {
        let problem = Arc::new(problem_def.into());
        let checker = Arc::new(validity_checker.clone());
        self.planner.setup(problem, checker);
    }

    pub fn solve(&mut self, timeout_secs: f32) -> Result<JsPath, String> {
        let timeout = Duration::from_secs_f32(timeout_secs);
        match self.planner.solve(timeout) {
            Ok(path) => Ok(JsPath { states: path }),
            Err(e) => Err(e.to_string()),
        }
    }
}
