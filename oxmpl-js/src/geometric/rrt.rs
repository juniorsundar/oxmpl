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
use oxmpl::geometric::RRT;
use std::sync::Arc;
use std::time::Duration;
use wasm_bindgen::prelude::*;

enum RrtVariant {
    RealVector(RRT<RealVectorState, RealVectorStateSpace, JsGoal>),
    SO2(RRT<SO2State, SO2StateSpace, JsGoal>),
    SO3(RRT<SO3State, SO3StateSpace, JsGoal>),
    Compound(RRT<CompoundState, CompoundStateSpace, JsGoal>),
    SE2(RRT<SE2State, SE2StateSpace, JsGoal>),
    SE3(RRT<SE3State, SE3StateSpace, JsGoal>),
}

#[wasm_bindgen(js_name = RRT)]
pub struct JsRRT {
    planner: RrtVariant,
    pd: ProblemDefinitionVariant,
}

#[wasm_bindgen(js_class = RRT)]
impl JsRRT {
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
                planner: RrtVariant::RealVector(RRT::new(max_distance, goal_bias, &planner_config)),
                pd: ProblemDefinitionVariant::RealVector(pd.clone()),
            },
            ProblemDefinitionVariant::SO2(pd) => Self {
                planner: RrtVariant::SO2(RRT::new(max_distance, goal_bias, &planner_config)),
                pd: ProblemDefinitionVariant::SO2(pd.clone()),
            },
            ProblemDefinitionVariant::SO3(pd) => Self {
                planner: RrtVariant::SO3(RRT::new(max_distance, goal_bias, &planner_config)),
                pd: ProblemDefinitionVariant::SO3(pd.clone()),
            },
            ProblemDefinitionVariant::Compound(pd) => Self {
                planner: RrtVariant::Compound(RRT::new(max_distance, goal_bias, &planner_config)),
                pd: ProblemDefinitionVariant::Compound(pd.clone()),
            },
            ProblemDefinitionVariant::SE2(pd) => Self {
                planner: RrtVariant::SE2(RRT::new(max_distance, goal_bias, &planner_config)),
                pd: ProblemDefinitionVariant::SE2(pd.clone()),
            },
            ProblemDefinitionVariant::SE3(pd) => Self {
                planner: RrtVariant::SE3(RRT::new(max_distance, goal_bias, &planner_config)),
                pd: ProblemDefinitionVariant::SE3(pd.clone()),
            },
        }
    }

    pub fn setup(&mut self, validity_checker: &JsStateValidityChecker) {
        let checker = Arc::new(validity_checker.clone());
        match &mut self.planner {
            RrtVariant::RealVector(p) => {
                if let ProblemDefinitionVariant::RealVector(pd) = &self.pd {
                    p.setup(pd.clone(), checker);
                }
            }
            RrtVariant::SO2(p) => {
                if let ProblemDefinitionVariant::SO2(pd) = &self.pd {
                    p.setup(pd.clone(), checker);
                }
            }
            RrtVariant::SO3(p) => {
                if let ProblemDefinitionVariant::SO3(pd) = &self.pd {
                    p.setup(pd.clone(), checker);
                }
            }
            RrtVariant::Compound(p) => {
                if let ProblemDefinitionVariant::Compound(pd) = &self.pd {
                    p.setup(pd.clone(), checker);
                }
            }
            RrtVariant::SE2(p) => {
                if let ProblemDefinitionVariant::SE2(pd) = &self.pd {
                    p.setup(pd.clone(), checker);
                }
            }
            RrtVariant::SE3(p) => {
                if let ProblemDefinitionVariant::SE3(pd) = &self.pd {
                    p.setup(pd.clone(), checker);
                }
            }
        }
    }

    pub fn solve(&mut self, timeout_secs: f32) -> Result<JsPath, String> {
        let timeout = Duration::from_secs_f32(timeout_secs);
        match &mut self.planner {
            RrtVariant::RealVector(p) => p
                .solve(timeout)
                .map(JsPath::from)
                .map_err(|e| e.to_string()),
            RrtVariant::SO2(p) => p
                .solve(timeout)
                .map(JsPath::from)
                .map_err(|e| e.to_string()),
            RrtVariant::SO3(p) => p
                .solve(timeout)
                .map(JsPath::from)
                .map_err(|e| e.to_string()),
            RrtVariant::Compound(p) => p
                .solve(timeout)
                .map(JsPath::from)
                .map_err(|e| e.to_string()),
            RrtVariant::SE2(p) => p
                .solve(timeout)
                .map(JsPath::from)
                .map_err(|e| e.to_string()),
            RrtVariant::SE3(p) => p
                .solve(timeout)
                .map(JsPath::from)
                .map_err(|e| e.to_string()),
        }
    }
}
