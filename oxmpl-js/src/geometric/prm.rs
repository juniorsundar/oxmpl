// Copyright (c) 2025 Ross Gardiner, Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::{
    goal::JsGoal,
    path::JsPath,
    planner::JsPlannerConfig,
    problem_definition::{JsProblemDefinition, ProblemDefinitionVariant},
    state_validity_checker::JsStateValidityChecker,
};
use oxmpl::base::{
    planner::Planner,
    space::{
        CompoundStateSpace, RealVectorStateSpace, SE2StateSpace, SE3StateSpace, SO2StateSpace,
        SO3StateSpace,
    },
    state::{CompoundState, RealVectorState, SE2State, SE3State, SO2State, SO3State},
};
use oxmpl::geometric::PRM;
use std::sync::Arc;
use std::time::Duration;
use wasm_bindgen::prelude::*;

enum PRMVariant {
    RealVector(PRM<RealVectorState, RealVectorStateSpace, JsGoal>),
    SO2(PRM<SO2State, SO2StateSpace, JsGoal>),
    SO3(PRM<SO3State, SO3StateSpace, JsGoal>),
    Compound(PRM<CompoundState, CompoundStateSpace, JsGoal>),
    SE2(PRM<SE2State, SE2StateSpace, JsGoal>),
    SE3(PRM<SE3State, SE3StateSpace, JsGoal>),
}

#[wasm_bindgen(js_name = PRM)]
pub struct JsPRM {
    planner: PRMVariant,
    pd: ProblemDefinitionVariant,
}

#[wasm_bindgen(js_class = PRM)]
impl JsPRM {
    #[wasm_bindgen(constructor)]
    pub fn new(
        timeout_secs: f32,
        connection_radius: f64,
        problem_def: &JsProblemDefinition,
        config: &JsPlannerConfig,
    ) -> Self {
        let planner_config = config.into();
        match &problem_def.inner {
            ProblemDefinitionVariant::RealVector(pd) => Self {
                planner: PRMVariant::RealVector(PRM::new(
                    timeout_secs.into(),
                    connection_radius,
                    &planner_config,
                )),
                pd: ProblemDefinitionVariant::RealVector(pd.clone()),
            },
            ProblemDefinitionVariant::SO2(pd) => Self {
                planner: PRMVariant::SO2(PRM::new(
                    timeout_secs.into(),
                    connection_radius,
                    &planner_config,
                )),
                pd: ProblemDefinitionVariant::SO2(pd.clone()),
            },
            ProblemDefinitionVariant::SO3(pd) => Self {
                planner: PRMVariant::SO3(PRM::new(
                    timeout_secs.into(),
                    connection_radius,
                    &planner_config,
                )),
                pd: ProblemDefinitionVariant::SO3(pd.clone()),
            },
            ProblemDefinitionVariant::Compound(pd) => Self {
                planner: PRMVariant::Compound(PRM::new(
                    timeout_secs.into(),
                    connection_radius,
                    &planner_config,
                )),
                pd: ProblemDefinitionVariant::Compound(pd.clone()),
            },
            ProblemDefinitionVariant::SE2(pd) => Self {
                planner: PRMVariant::SE2(PRM::new(
                    timeout_secs.into(),
                    connection_radius,
                    &planner_config,
                )),
                pd: ProblemDefinitionVariant::SE2(pd.clone()),
            },
            ProblemDefinitionVariant::SE3(pd) => Self {
                planner: PRMVariant::SE3(PRM::new(
                    timeout_secs.into(),
                    connection_radius,
                    &planner_config,
                )),
                pd: ProblemDefinitionVariant::SE3(pd.clone()),
            },
        }
    }

    pub fn setup(&mut self, validity_checker: &JsStateValidityChecker) {
        let checker = Arc::new(validity_checker.clone());
        match &mut self.planner {
            PRMVariant::RealVector(p) => {
                if let ProblemDefinitionVariant::RealVector(pd) = &self.pd {
                    p.setup(pd.clone(), checker);
                }
            }
            PRMVariant::SO2(p) => {
                if let ProblemDefinitionVariant::SO2(pd) = &self.pd {
                    p.setup(pd.clone(), checker);
                }
            }
            PRMVariant::SO3(p) => {
                if let ProblemDefinitionVariant::SO3(pd) = &self.pd {
                    p.setup(pd.clone(), checker);
                }
            }
            PRMVariant::Compound(p) => {
                if let ProblemDefinitionVariant::Compound(pd) = &self.pd {
                    p.setup(pd.clone(), checker);
                }
            }
            PRMVariant::SE2(p) => {
                if let ProblemDefinitionVariant::SE2(pd) = &self.pd {
                    p.setup(pd.clone(), checker);
                }
            }
            PRMVariant::SE3(p) => {
                if let ProblemDefinitionVariant::SE3(pd) = &self.pd {
                    p.setup(pd.clone(), checker);
                }
            }
        }
    }

    #[wasm_bindgen(js_name = constructRoadmap)]
    pub fn construct_roadmap(&mut self) -> Result<(), String> {
        match &mut self.planner {
            PRMVariant::RealVector(p) => p.construct_roadmap().map_err(|e| e.to_string()),
            PRMVariant::SO2(p) => p.construct_roadmap().map_err(|e| e.to_string()),
            PRMVariant::SO3(p) => p.construct_roadmap().map_err(|e| e.to_string()),
            PRMVariant::Compound(p) => p.construct_roadmap().map_err(|e| e.to_string()),
            PRMVariant::SE2(p) => p.construct_roadmap().map_err(|e| e.to_string()),
            PRMVariant::SE3(p) => p.construct_roadmap().map_err(|e| e.to_string()),
        }
    }

    pub fn solve(&mut self, timeout_secs: f32) -> Result<JsPath, String> {
        let timeout = Duration::from_secs_f32(timeout_secs);
        match &mut self.planner {
            PRMVariant::RealVector(p) => p
                .solve(timeout)
                .map(JsPath::from)
                .map_err(|e| e.to_string()),
            PRMVariant::SO2(p) => p
                .solve(timeout)
                .map(JsPath::from)
                .map_err(|e| e.to_string()),
            PRMVariant::SO3(p) => p
                .solve(timeout)
                .map(JsPath::from)
                .map_err(|e| e.to_string()),
            PRMVariant::Compound(p) => p
                .solve(timeout)
                .map(JsPath::from)
                .map_err(|e| e.to_string()),
            PRMVariant::SE2(p) => p
                .solve(timeout)
                .map(JsPath::from)
                .map_err(|e| e.to_string()),
            PRMVariant::SE3(p) => p
                .solve(timeout)
                .map(JsPath::from)
                .map_err(|e| e.to_string()),
        }
    }
}
