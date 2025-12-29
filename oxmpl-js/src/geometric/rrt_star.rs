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
use oxmpl::geometric::RRTStar;
use std::sync::Arc;
use std::time::Duration;
use wasm_bindgen::prelude::*;

enum RRTStarVariant {
    RealVector(RRTStar<RealVectorState, RealVectorStateSpace, JsGoal>),
    SO2(RRTStar<SO2State, SO2StateSpace, JsGoal>),
    SO3(RRTStar<SO3State, SO3StateSpace, JsGoal>),
    Compound(RRTStar<CompoundState, CompoundStateSpace, JsGoal>),
    SE2(RRTStar<SE2State, SE2StateSpace, JsGoal>),
    SE3(RRTStar<SE3State, SE3StateSpace, JsGoal>),
}

#[wasm_bindgen(js_name = RRTStar)]
pub struct JsRRTStar {
    planner: RRTStarVariant,
    pd: ProblemDefinitionVariant,
}

#[wasm_bindgen(js_class = RRTStar)]
impl JsRRTStar {
    #[wasm_bindgen(constructor)]
    pub fn new(
        max_distance: f64,
        goal_bias: f64,
        search_radius: f64,
        problem_def: &JsProblemDefinition,
        config: &JsPlannerConfig,
    ) -> Self {
        let planner_config = config.into();
        match &problem_def.inner {
            ProblemDefinitionVariant::RealVector(pd) => Self {
                planner: RRTStarVariant::RealVector(RRTStar::new(
                    max_distance,
                    goal_bias,
                    search_radius,
                    &planner_config,
                )),
                pd: ProblemDefinitionVariant::RealVector(pd.clone()),
            },
            ProblemDefinitionVariant::SO2(pd) => Self {
                planner: RRTStarVariant::SO2(RRTStar::new(
                    max_distance,
                    goal_bias,
                    search_radius,
                    &planner_config,
                )),
                pd: ProblemDefinitionVariant::SO2(pd.clone()),
            },
            ProblemDefinitionVariant::SO3(pd) => Self {
                planner: RRTStarVariant::SO3(RRTStar::new(
                    max_distance,
                    goal_bias,
                    search_radius,
                    &planner_config,
                )),
                pd: ProblemDefinitionVariant::SO3(pd.clone()),
            },
            ProblemDefinitionVariant::Compound(pd) => Self {
                planner: RRTStarVariant::Compound(RRTStar::new(
                    max_distance,
                    goal_bias,
                    search_radius,
                    &planner_config,
                )),
                pd: ProblemDefinitionVariant::Compound(pd.clone()),
            },
            ProblemDefinitionVariant::SE2(pd) => Self {
                planner: RRTStarVariant::SE2(RRTStar::new(
                    max_distance,
                    goal_bias,
                    search_radius,
                    &planner_config,
                )),
                pd: ProblemDefinitionVariant::SE2(pd.clone()),
            },
            ProblemDefinitionVariant::SE3(pd) => Self {
                planner: RRTStarVariant::SE3(RRTStar::new(
                    max_distance,
                    goal_bias,
                    search_radius,
                    &planner_config,
                )),
                pd: ProblemDefinitionVariant::SE3(pd.clone()),
            },
        }
    }

    pub fn setup(&mut self, validity_checker: &JsStateValidityChecker) {
        let checker = Arc::new(validity_checker.clone());
        match &mut self.planner {
            RRTStarVariant::RealVector(p) => {
                if let ProblemDefinitionVariant::RealVector(pd) = &self.pd {
                    p.setup(pd.clone(), checker);
                }
            }
            RRTStarVariant::SO2(p) => {
                if let ProblemDefinitionVariant::SO2(pd) = &self.pd {
                    p.setup(pd.clone(), checker);
                }
            }
            RRTStarVariant::SO3(p) => {
                if let ProblemDefinitionVariant::SO3(pd) = &self.pd {
                    p.setup(pd.clone(), checker);
                }
            }
            RRTStarVariant::Compound(p) => {
                if let ProblemDefinitionVariant::Compound(pd) = &self.pd {
                    p.setup(pd.clone(), checker);
                }
            }
            RRTStarVariant::SE2(p) => {
                if let ProblemDefinitionVariant::SE2(pd) = &self.pd {
                    p.setup(pd.clone(), checker);
                }
            }
            RRTStarVariant::SE3(p) => {
                if let ProblemDefinitionVariant::SE3(pd) = &self.pd {
                    p.setup(pd.clone(), checker);
                }
            }
        }
    }

    pub fn solve(&mut self, timeout_secs: f32) -> Result<JsPath, String> {
        let timeout = Duration::from_secs_f32(timeout_secs);
        match &mut self.planner {
            RRTStarVariant::RealVector(p) => p
                .solve(timeout)
                .map(JsPath::from)
                .map_err(|e| e.to_string()),
            RRTStarVariant::SO2(p) => p
                .solve(timeout)
                .map(JsPath::from)
                .map_err(|e| e.to_string()),
            RRTStarVariant::SO3(p) => p
                .solve(timeout)
                .map(JsPath::from)
                .map_err(|e| e.to_string()),
            RRTStarVariant::Compound(p) => p
                .solve(timeout)
                .map(JsPath::from)
                .map_err(|e| e.to_string()),
            RRTStarVariant::SE2(p) => p
                .solve(timeout)
                .map(JsPath::from)
                .map_err(|e| e.to_string()),
            RRTStarVariant::SE3(p) => p
                .solve(timeout)
                .map(JsPath::from)
                .map_err(|e| e.to_string()),
        }
    }
}
