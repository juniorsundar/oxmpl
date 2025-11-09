// Copyright (c) 2025 Ross Gardiner, Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::js_state_convert::*;
use oxmpl::base::{state::RealVectorState, validity::StateValidityChecker};
use wasm_bindgen::prelude::*;
use web_sys::console;

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
