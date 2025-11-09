// Copyright (c) 2025 Ross Gardiner, Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::js_state_convert::*;
use js_sys::Float64Array;
use oxmpl::base::{
    error::StateSamplingError,
    goal::{Goal, GoalRegion, GoalSampleableRegion},
    state::RealVectorState,
};
use wasm_bindgen::prelude::*;
use web_sys::console;

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
