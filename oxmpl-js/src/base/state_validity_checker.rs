// Copyright (c) 2025 Ross Gardiner, Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::js_state_convert::*;
use oxmpl::base::{
    state::{CompoundState, RealVectorState, SE2State, SE3State, SO2State, SO3State, State},
    validity::StateValidityChecker,
};
use wasm_bindgen::prelude::*;
use web_sys::console;

#[wasm_bindgen(js_name = StateValidityChecker)]
#[derive(Clone)]
pub struct JsStateValidityChecker {
    callback: js_sys::Function,
}

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(typescript_type = "(state: any) => boolean")]
    pub type StateValidityCallback;
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

impl JsStateValidityChecker {
    fn call_is_valid<S: JsStateConvert + State>(&self, state: &S) -> bool {
        let js_state = state.to_js_value();

        match self.callback.call1(&JsValue::NULL, &js_state) {
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

impl StateValidityChecker<RealVectorState> for JsStateValidityChecker {
    fn is_valid(&self, state: &RealVectorState) -> bool {
        self.call_is_valid(state)
    }
}

impl StateValidityChecker<SO2State> for JsStateValidityChecker {
    fn is_valid(&self, state: &SO2State) -> bool {
        self.call_is_valid(state)
    }
}

impl StateValidityChecker<SO3State> for JsStateValidityChecker {
    fn is_valid(&self, state: &SO3State) -> bool {
        self.call_is_valid(state)
    }
}

impl StateValidityChecker<SE2State> for JsStateValidityChecker {
    fn is_valid(&self, state: &SE2State) -> bool {
        self.call_is_valid(state)
    }
}

impl StateValidityChecker<SE3State> for JsStateValidityChecker {
    fn is_valid(&self, state: &SE3State) -> bool {
        self.call_is_valid(state)
    }
}

impl StateValidityChecker<CompoundState> for JsStateValidityChecker {
    fn is_valid(&self, state: &CompoundState) -> bool {
        self.call_is_valid(state)
    }
}
