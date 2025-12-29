// Copyright (c) 2025 Ross Gardiner, Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::js_state_convert::*;
use oxmpl::base::{
    error::StateSamplingError,
    goal::{Goal, GoalRegion, GoalSampleableRegion},
    state::{CompoundState, RealVectorState, SE2State, SE3State, SO2State, SO3State, State},
};
use wasm_bindgen::prelude::*;
use web_sys::console;

#[wasm_bindgen(js_name = Goal)]
#[derive(Clone)]
pub struct JsGoal {
    pub(crate) instance: JsValue,
}

#[wasm_bindgen(js_class = Goal)]
impl JsGoal {
    #[wasm_bindgen(constructor)]
    pub fn new(instance: JsValue) -> Self {
        Self { instance }
    }
}

// Helper for calling JS methods
impl JsGoal {
    fn call_is_satisfied<S: JsStateConvert + State>(&self, state: &S) -> bool {
        let js_state = state.to_js_value();
        match js_sys::Reflect::get(&self.instance, &JsValue::from_str("isSatisfied")) {
            Ok(func_val) => {
                if let Ok(func) = func_val.dyn_into::<js_sys::Function>() {
                    match func.call1(&self.instance, &js_state) {
                        Ok(result) => result.as_bool().unwrap_or(false),
                        Err(e) => {
                            console::error_2(&"Goal.isSatisfied failed:".into(), &e);
                            false
                        }
                    }
                } else {
                    false
                }
            }
            Err(_) => false,
        }
    }

    fn call_distance_goal<S: JsStateConvert + State>(&self, state: &S) -> f64 {
        let js_state = state.to_js_value();
        match js_sys::Reflect::get(&self.instance, &JsValue::from_str("distanceGoal")) {
            Ok(func_val) => {
                if let Ok(func) = func_val.dyn_into::<js_sys::Function>() {
                    match func.call1(&self.instance, &js_state) {
                        Ok(result) => result.as_f64().unwrap_or(f64::INFINITY),
                        Err(e) => {
                            console::error_2(&"Goal.distanceGoal failed:".into(), &e);
                            f64::INFINITY
                        }
                    }
                } else {
                    f64::INFINITY
                }
            }
            Err(_) => f64::INFINITY,
        }
    }

    fn call_sample_goal<S: JsStateConvert + State>(&self) -> Result<S, StateSamplingError> {
        match js_sys::Reflect::get(&self.instance, &JsValue::from_str("sampleGoal")) {
            Ok(func_val) => {
                if let Ok(func) = func_val.dyn_into::<js_sys::Function>() {
                    match func.call0(&self.instance) {
                        Ok(result) => S::from_js_value(result)
                            .map_err(|_| StateSamplingError::GoalRegionUnsatisfiable),
                        Err(e) => {
                            console::error_2(&"Goal.sampleGoal failed:".into(), &e);
                            Err(StateSamplingError::GoalRegionUnsatisfiable)
                        }
                    }
                } else {
                    Err(StateSamplingError::GoalRegionUnsatisfiable)
                }
            }
            Err(_) => Err(StateSamplingError::GoalRegionUnsatisfiable),
        }
    }
}

// Implementations for each state type
impl Goal<RealVectorState> for JsGoal {
    fn is_satisfied(&self, state: &RealVectorState) -> bool {
        self.call_is_satisfied(state)
    }
}
impl GoalRegion<RealVectorState> for JsGoal {
    fn distance_goal(&self, state: &RealVectorState) -> f64 {
        self.call_distance_goal(state)
    }
}
impl GoalSampleableRegion<RealVectorState> for JsGoal {
    fn sample_goal(
        &self,
        _rng: &mut impl rand::Rng,
    ) -> Result<RealVectorState, StateSamplingError> {
        self.call_sample_goal()
    }
}

impl Goal<SO2State> for JsGoal {
    fn is_satisfied(&self, state: &SO2State) -> bool {
        self.call_is_satisfied(state)
    }
}
impl GoalRegion<SO2State> for JsGoal {
    fn distance_goal(&self, state: &SO2State) -> f64 {
        self.call_distance_goal(state)
    }
}
impl GoalSampleableRegion<SO2State> for JsGoal {
    fn sample_goal(&self, _rng: &mut impl rand::Rng) -> Result<SO2State, StateSamplingError> {
        self.call_sample_goal()
    }
}

impl Goal<SO3State> for JsGoal {
    fn is_satisfied(&self, state: &SO3State) -> bool {
        self.call_is_satisfied(state)
    }
}
impl GoalRegion<SO3State> for JsGoal {
    fn distance_goal(&self, state: &SO3State) -> f64 {
        self.call_distance_goal(state)
    }
}
impl GoalSampleableRegion<SO3State> for JsGoal {
    fn sample_goal(&self, _rng: &mut impl rand::Rng) -> Result<SO3State, StateSamplingError> {
        self.call_sample_goal()
    }
}

impl Goal<SE2State> for JsGoal {
    fn is_satisfied(&self, state: &SE2State) -> bool {
        self.call_is_satisfied(state)
    }
}
impl GoalRegion<SE2State> for JsGoal {
    fn distance_goal(&self, state: &SE2State) -> f64 {
        self.call_distance_goal(state)
    }
}
impl GoalSampleableRegion<SE2State> for JsGoal {
    fn sample_goal(&self, _rng: &mut impl rand::Rng) -> Result<SE2State, StateSamplingError> {
        self.call_sample_goal()
    }
}

impl Goal<SE3State> for JsGoal {
    fn is_satisfied(&self, state: &SE3State) -> bool {
        self.call_is_satisfied(state)
    }
}
impl GoalRegion<SE3State> for JsGoal {
    fn distance_goal(&self, state: &SE3State) -> f64 {
        self.call_distance_goal(state)
    }
}
impl GoalSampleableRegion<SE3State> for JsGoal {
    fn sample_goal(&self, _rng: &mut impl rand::Rng) -> Result<SE3State, StateSamplingError> {
        self.call_sample_goal()
    }
}

impl Goal<CompoundState> for JsGoal {
    fn is_satisfied(&self, state: &CompoundState) -> bool {
        self.call_is_satisfied(state)
    }
}
impl GoalRegion<CompoundState> for JsGoal {
    fn distance_goal(&self, state: &CompoundState) -> f64 {
        self.call_distance_goal(state)
    }
}
impl GoalSampleableRegion<CompoundState> for JsGoal {
    fn sample_goal(&self, _rng: &mut impl rand::Rng) -> Result<CompoundState, StateSamplingError> {
        self.call_sample_goal()
    }
}
