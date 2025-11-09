// Copyright (c) 2025 Ross Gardiner, Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::js_state_convert::*;
use js_sys::Float64Array;
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
    pub fn get_states(&self) -> Vec<Float64Array> {
        self.states.0.iter().map(state_to_js_array).collect()
    }

    pub fn length(&self) -> usize {
        self.states.0.len()
    }
}
