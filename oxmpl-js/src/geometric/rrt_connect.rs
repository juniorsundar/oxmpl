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
use oxmpl::geometric::RRTConnect;
use std::sync::Arc;
use std::time::Duration;
use wasm_bindgen::prelude::*;

enum RRTConnectVariant {
    RealVector(RRTConnect<RealVectorState, RealVectorStateSpace, JsGoal>),
    SO2(RRTConnect<SO2State, SO2StateSpace, JsGoal>),
    SO3(RRTConnect<SO3State, SO3StateSpace, JsGoal>),
    Compound(RRTConnect<CompoundState, CompoundStateSpace, JsGoal>),
    SE2(RRTConnect<SE2State, SE2StateSpace, JsGoal>),
    SE3(RRTConnect<SE3State, SE3StateSpace, JsGoal>),
}

#[wasm_bindgen(js_name = RRTConnect)]
pub struct JsRRTConnect {
    planner: RRTConnectVariant,
    pd: ProblemDefinitionVariant,
}

#[wasm_bindgen(js_class = RRTConnect)]
impl JsRRTConnect {
    #[wasm_bindgen(constructor)]
    pub fn new(
        max_distance: f64,
        goal_bias: f64,
        problem_def: &JsProblemDefinition,
        config: &JsPlannerConfig,
    ) -> Self {
        let planner_config = config.into();
        match &problem_def.inner {
            ProblemDefinitionVariant::RealVector(pd) => Self {
                planner: RRTConnectVariant::RealVector(RRTConnect::new(
                    max_distance,
                    goal_bias,
                    &planner_config,
                )),
                pd: ProblemDefinitionVariant::RealVector(pd.clone()),
            },
            ProblemDefinitionVariant::SO2(pd) => Self {
                planner: RRTConnectVariant::SO2(RRTConnect::new(
                    max_distance,
                    goal_bias,
                    &planner_config,
                )),
                pd: ProblemDefinitionVariant::SO2(pd.clone()),
            },
            ProblemDefinitionVariant::SO3(pd) => Self {
                planner: RRTConnectVariant::SO3(RRTConnect::new(
                    max_distance,
                    goal_bias,
                    &planner_config,
                )),
                pd: ProblemDefinitionVariant::SO3(pd.clone()),
            },
            ProblemDefinitionVariant::Compound(pd) => Self {
                planner: RRTConnectVariant::Compound(RRTConnect::new(
                    max_distance,
                    goal_bias,
                    &planner_config,
                )),
                pd: ProblemDefinitionVariant::Compound(pd.clone()),
            },
            ProblemDefinitionVariant::SE2(pd) => Self {
                planner: RRTConnectVariant::SE2(RRTConnect::new(
                    max_distance,
                    goal_bias,
                    &planner_config,
                )),
                pd: ProblemDefinitionVariant::SE2(pd.clone()),
            },
            ProblemDefinitionVariant::SE3(pd) => Self {
                planner: RRTConnectVariant::SE3(RRTConnect::new(
                    max_distance,
                    goal_bias,
                    &planner_config,
                )),
                pd: ProblemDefinitionVariant::SE3(pd.clone()),
            },
        }
    }

    pub fn setup(&mut self, validity_checker: &JsStateValidityChecker) {
        let checker = Arc::new(validity_checker.clone());
        match &mut self.planner {
            RRTConnectVariant::RealVector(p) => {
                if let ProblemDefinitionVariant::RealVector(pd) = &self.pd {
                    p.setup(pd.clone(), checker);
                }
            }
            RRTConnectVariant::SO2(p) => {
                if let ProblemDefinitionVariant::SO2(pd) = &self.pd {
                    p.setup(pd.clone(), checker);
                }
            }
            RRTConnectVariant::SO3(p) => {
                if let ProblemDefinitionVariant::SO3(pd) = &self.pd {
                    p.setup(pd.clone(), checker);
                }
            }
            RRTConnectVariant::Compound(p) => {
                if let ProblemDefinitionVariant::Compound(pd) = &self.pd {
                    p.setup(pd.clone(), checker);
                }
            }
            RRTConnectVariant::SE2(p) => {
                if let ProblemDefinitionVariant::SE2(pd) = &self.pd {
                    p.setup(pd.clone(), checker);
                }
            }
            RRTConnectVariant::SE3(p) => {
                if let ProblemDefinitionVariant::SE3(pd) = &self.pd {
                    p.setup(pd.clone(), checker);
                }
            }
        }
    }

    pub fn solve(&mut self, timeout_secs: f32) -> Result<JsPath, String> {
        let timeout = Duration::from_secs_f32(timeout_secs);
        match &mut self.planner {
            RRTConnectVariant::RealVector(p) => p
                .solve(timeout)
                .map(JsPath::from)
                .map_err(|e| e.to_string()),
            RRTConnectVariant::SO2(p) => p
                .solve(timeout)
                .map(JsPath::from)
                .map_err(|e| e.to_string()),
            RRTConnectVariant::SO3(p) => p
                .solve(timeout)
                .map(JsPath::from)
                .map_err(|e| e.to_string()),
            RRTConnectVariant::Compound(p) => p
                .solve(timeout)
                .map(JsPath::from)
                .map_err(|e| e.to_string()),
            RRTConnectVariant::SE2(p) => p
                .solve(timeout)
                .map(JsPath::from)
                .map_err(|e| e.to_string()),
            RRTConnectVariant::SE3(p) => p
                .solve(timeout)
                .map(JsPath::from)
                .map_err(|e| e.to_string()),
        }
    }
}
