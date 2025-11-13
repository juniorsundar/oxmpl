// Copyright (c) 2025 Ross Gardiner, Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use js_sys::Array;
use oxmpl::base::{planner::Path, state::RealVectorState};
use wasm_bindgen::prelude::*;

#[wasm_bindgen(js_name = Path)]
pub struct JsPath {
    #[wasm_bindgen(skip)]
    pub states: Path<RealVectorState>,
}

#[wasm_bindgen(js_class = Path)]
impl JsPath {
    #[wasm_bindgen(js_name = getStates)]
    pub fn get_states(&self) -> Array {
        self.states
            .0
            .iter()
            .map(|s| {
                s.values
                    .iter()
                    .map(|&v| JsValue::from_f64(v))
                    .collect::<Array>()
            })
            .collect::<Array>()
    }

    #[wasm_bindgen(js_name = getLength)]
    pub fn get_length(&self) -> usize {
        self.states.0.len()
    }
}
