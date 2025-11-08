// Copyright (c) 2025 Ross Gardiner
//
// SPDX-License-Identifier: BSD-3-Clause

#![allow(clippy::arc_with_non_send_sync)]

use js_sys::Float64Array;
use oxmpl::base::{
    error::StateSamplingError,
    goal::{Goal, GoalRegion, GoalSampleableRegion},
    planner::{Path, Planner, PlannerConfig},
    problem_definition::ProblemDefinition,
    space::{RealVectorStateSpace, StateSpace},
    state::RealVectorState,
    validity::StateValidityChecker,
};
use oxmpl::geometric::{RRTConnect, RRTStar, PRM, RRT};
use rand::rng;
use std::sync::Arc;
use std::time::Duration;
use wasm_bindgen::prelude::*;
use web_sys::console;

// Helper functions for converting between Rust states and JavaScript arrays
fn state_to_js_array(state: &RealVectorState) -> Float64Array {
    let array = Float64Array::new_with_length(state.values.len() as u32);
    for (i, &val) in state.values.iter().enumerate() {
        array.set_index(i as u32, val);
    }
    array
}

fn js_array_to_state(array: &Float64Array) -> RealVectorState {
    let mut values = Vec::new();
    for i in 0..array.length() {
        values.push(array.get_index(i));
    }
    RealVectorState::new(values)
}

// Set panic hook to get better error messages
#[wasm_bindgen(start)]
pub fn set_panic_hook() {
    console_error_panic_hook::set_once();
}

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = console)]
    fn log(s: &str);
}

#[wasm_bindgen(js_name = StateValidityChecker)]
#[derive(Clone)]
pub struct JsStateValidityChecker {
    callback: js_sys::Function,
}

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(typescript_type = "(state: Float64Array) => boolean")]
    pub type StateValidityCallback;

    #[wasm_bindgen(typescript_type = "Array<Float64Array>")]
    pub type StateArray;
}

#[wasm_bindgen(js_class = StateValidityChecker)]
impl JsStateValidityChecker {
    #[wasm_bindgen(constructor)]
    pub fn new(callback: StateValidityCallback) -> Self {
        Self {
            callback: JsValue::from(callback).into(),
        }
    }
}

impl StateValidityChecker<RealVectorState> for JsStateValidityChecker {
    fn is_valid(&self, state: &RealVectorState) -> bool {
        let array = state_to_js_array(state);

        match self.callback.call1(&JsValue::NULL, &array) {
            Ok(result) => match result.as_bool() {
                Some(is_valid) => is_valid,
                None => {
                    console::warn_1(&"State validity checker returned non-boolean value".into());
                    false
                }
            },
            Err(e) => {
                console::error_2(&"State validity checker callback failed:".into(), &e);
                false
            }
        }
    }
}

#[wasm_bindgen(js_name = Goal)]
#[derive(Clone)]
pub struct JsGoal {
    is_satisfied_fn: js_sys::Function,
    distance_goal_fn: js_sys::Function,
    sample_goal_fn: js_sys::Function,
}

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(typescript_type = "(state: Float64Array) => boolean")]
    pub type GoalSatisfactionCallback;

    #[wasm_bindgen(typescript_type = "(state: Float64Array) => number")]
    pub type GoalDistanceCallback;

    #[wasm_bindgen(typescript_type = "() => Float64Array")]
    pub type GoalSampleCallback;
}

#[wasm_bindgen(js_class = Goal)]
impl JsGoal {
    #[wasm_bindgen(constructor)]
    pub fn new(
        is_satisfied_fn: GoalSatisfactionCallback,
        distance_goal_fn: GoalDistanceCallback,
        sample_goal_fn: GoalSampleCallback,
    ) -> Self {
        Self {
            is_satisfied_fn: JsValue::from(is_satisfied_fn).into(),
            distance_goal_fn: JsValue::from(distance_goal_fn).into(),
            sample_goal_fn: JsValue::from(sample_goal_fn).into(),
        }
    }
}

impl Goal<RealVectorState> for JsGoal {
    fn is_satisfied(&self, state: &RealVectorState) -> bool {
        let array = state_to_js_array(state);

        match self.is_satisfied_fn.call1(&JsValue::NULL, &array) {
            Ok(result) => match result.as_bool() {
                Some(satisfied) => satisfied,
                None => {
                    console::warn_1(&"Goal satisfaction checker returned non-boolean value".into());
                    false
                }
            },
            Err(e) => {
                console::error_2(&"Goal satisfaction checker callback failed:".into(), &e);
                false
            }
        }
    }
}

impl GoalRegion<RealVectorState> for JsGoal {
    fn distance_goal(&self, state: &RealVectorState) -> f64 {
        let array = state_to_js_array(state);

        match self.distance_goal_fn.call1(&JsValue::NULL, &array) {
            Ok(result) => match result.as_f64() {
                Some(distance) => distance,
                None => {
                    console::warn_1(&"Goal distance function returned non-numeric value".into());
                    f64::INFINITY
                }
            },
            Err(e) => {
                console::error_2(&"Goal distance function callback failed:".into(), &e);
                f64::INFINITY
            }
        }
    }
}

impl GoalSampleableRegion<RealVectorState> for JsGoal {
    fn sample_goal(
        &self,
        _rng: &mut impl rand::Rng,
    ) -> Result<RealVectorState, StateSamplingError> {
        match self.sample_goal_fn.call0(&JsValue::NULL) {
            Ok(result) => {
                if let Ok(array) = result.dyn_into::<Float64Array>() {
                    Ok(js_array_to_state(&array))
                } else {
                    Err(StateSamplingError::GoalRegionUnsatisfiable)
                }
            }
            Err(_) => Err(StateSamplingError::GoalRegionUnsatisfiable),
        }
    }
}

#[wasm_bindgen(js_name = RealVectorStateSpace)]
pub struct JsRealVectorStateSpace {
    inner: Arc<RealVectorStateSpace>,
}

#[wasm_bindgen(js_class = RealVectorStateSpace)]
impl JsRealVectorStateSpace {
    #[wasm_bindgen(constructor)]
    pub fn new(
        dimension: usize,
        bounds: Option<Vec<f64>>,
    ) -> Result<JsRealVectorStateSpace, String> {
        let bounds_vec = if let Some(b) = bounds {
            if b.len() != dimension * 2 {
                return Err(format!(
                    "Bounds array must have {} elements (2 per dimension)",
                    dimension * 2
                ));
            }
            let mut bounds_tuples = Vec::new();
            for i in 0..dimension {
                bounds_tuples.push((b[i * 2], b[i * 2 + 1]));
            }
            Some(bounds_tuples)
        } else {
            None
        };

        match RealVectorStateSpace::new(dimension, bounds_vec) {
            Ok(space) => Ok(Self {
                inner: Arc::new(space),
            }),
            Err(e) => Err(e.to_string()),
        }
    }

    pub fn sample(&self) -> Result<Vec<f64>, String> {
        let mut rng = rng();
        match self.inner.sample_uniform(&mut rng) {
            Ok(state) => Ok(state.values),
            Err(e) => Err(e.to_string()),
        }
    }

    pub fn distance(&self, state1: Vec<f64>, state2: Vec<f64>) -> f64 {
        let s1 = RealVectorState::new(state1);
        let s2 = RealVectorState::new(state2);
        self.inner.distance(&s1, &s2)
    }

    #[wasm_bindgen(js_name = getDimension)]
    pub fn get_dimension(&self) -> usize {
        self.inner.dimension
    }
}

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

#[wasm_bindgen(js_name = Path)]
pub struct JsPath {
    states: Path<RealVectorState>,
}

#[wasm_bindgen(js_class = Path)]
impl JsPath {
    #[wasm_bindgen(js_name = getStates)]
    pub fn get_states(&self) -> Vec<Float64Array> {
        self.states.0.iter().map(state_to_js_array).collect()
    }

    pub fn length(&self) -> usize {
        self.states.0.len()
    }
}

#[wasm_bindgen(js_name = PlannerConfig)]
pub struct JsPlannerConfig {
    seed: Option<u64>,
}

#[wasm_bindgen(js_class = PlannerConfig)]
impl JsPlannerConfig {
    #[wasm_bindgen(constructor)]
    pub fn new(seed: Option<f64>) -> Self {
        Self {
            seed: seed.map(|s| s as u64),
        }
    }
}

impl From<&JsPlannerConfig> for PlannerConfig {
    fn from(js_planner_config: &JsPlannerConfig) -> Self {
        PlannerConfig {
            seed: (js_planner_config.seed),
        }
    }
}

#[wasm_bindgen(js_name = RRT)]
pub struct JsRRT {
    planner: RRT<RealVectorState, RealVectorStateSpace, JsGoal>,
}

#[wasm_bindgen(js_class = RRT)]
impl JsRRT {
    /**
     * @param {number} max_distance
     * @param {number} goal_bias
     * @param {JsPlannerConfig} config
     */
    #[wasm_bindgen(constructor)]
    pub fn new(max_distance: f32, goal_bias: f32, config: &JsPlannerConfig) -> Self {
        Self {
            planner: RRT::new(max_distance as f64, goal_bias as f64, &config.into()),
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

#[wasm_bindgen(js_name = RRTStar)]
pub struct JsRRTStar {
    planner: RRTStar<RealVectorState, RealVectorStateSpace, JsGoal>,
}

#[wasm_bindgen(js_class = RRTStar)]
impl JsRRTStar {
    /**
     * @param {number} max_distance
     * @param {number} goal_bias
     * @param {number} search_radius
     * @param {JsPlannerConfig} config
     */
    #[wasm_bindgen(constructor)]
    pub fn new(
        max_distance: f32,
        goal_bias: f32,
        search_radius: f32,
        config: &JsPlannerConfig,
    ) -> Self {
        Self {
            planner: RRTStar::new(
                max_distance as f64,
                goal_bias as f64,
                search_radius as f64,
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

    pub fn solve(&mut self, timeout_secs: f32) -> Result<JsPath, String> {
        let timeout = Duration::from_secs_f32(timeout_secs);
        match self.planner.solve(timeout) {
            Ok(path) => Ok(JsPath { states: path }),
            Err(e) => Err(e.to_string()),
        }
    }
}

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
