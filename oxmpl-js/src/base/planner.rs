// Copyright (c) 2025 Ross Gardiner, Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use oxmpl::base::planner::PlannerConfig;
use wasm_bindgen::prelude::*;

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
