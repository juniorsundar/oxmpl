// Copyright (c) 2025 Ross Gardiner, Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::{
    goal::JsGoal, path::JsPath, planner::JsPlannerConfig, problem_definition::JsProblemDefinition,
    state_validity_checker::JsStateValidityChecker,
};
use oxmpl::base::{planner::Planner, space::RealVectorStateSpace, state::RealVectorState};
use oxmpl::geometric::PRM;
use std::sync::Arc;
use std::time::Duration;
use wasm_bindgen::prelude::*;

#[wasm_bindgen(js_name = PRM)]
pub struct JsPRM {
    planner: PRM<RealVectorState, RealVectorStateSpace, JsGoal>,
}

#[wasm_bindgen(js_class = PRM)]
impl JsPRM {
    /**
     * @param {number} timeout_secs
     * @param {number} connection_radius
     * @param {JsPlannerConfig} config
     */
    #[wasm_bindgen(constructor)]
    pub fn new(timeout_secs: f32, connection_radius: f32, config: &JsPlannerConfig) -> Self {
        Self {
            planner: PRM::new(
                timeout_secs.into(),
                connection_radius as f64,
                &config.into(),
            ),
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

    #[wasm_bindgen(js_name = constructRoadmap)]
    pub fn construct_roadmap(&mut self) -> Result<(), String> {
        match self.planner.construct_roadmap() {
            Ok(()) => Ok(()),
            Err(e) => Err(e.to_string()),
        }
    }

    pub fn solve(&mut self, timeout_secs: f32) -> Result<JsPath, String> {
        let timeout = Duration::from_secs_f32(timeout_secs);
        match self.planner.solve(timeout) {
            Ok(path) => Ok(JsPath { states: path }),
            Err(e) => Err(e.to_string()),
        }
    }
}
