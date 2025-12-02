// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use std::sync::Arc;

use oxmpl::base::state::SE2State as OxmplSE2State;
use wasm_bindgen::prelude::*;

use crate::base::{JsRealVectorState, JsSO2State};

#[wasm_bindgen(js_name = SE2State)]
pub struct JsSE2State {
    #[wasm_bindgen(skip)]
    pub inner: Arc<OxmplSE2State>,
}

#[wasm_bindgen(js_class = SE2State)]
impl JsSE2State {
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64, yaw: f64) -> Self {
        let state = OxmplSE2State::new(x, y, yaw);
        Self {
            inner: Arc::new(state),
        }
    }

    #[wasm_bindgen(getter)]
    pub fn x(&self) -> f64 {
        self.inner.get_x()
    }
    #[wasm_bindgen(getter)]
    pub fn y(&self) -> f64 {
        self.inner.get_y()
    }
    #[wasm_bindgen(getter)]
    pub fn yaw(&self) -> f64 {
        self.inner.get_yaw()
    }
    #[wasm_bindgen(getter)]
    pub fn translation(&self) -> JsRealVectorState {
        let translation_rs = self.inner.get_translation();
        JsRealVectorState::new(translation_rs.values.clone())
    }
    #[wasm_bindgen(getter)]
    pub fn rotation(&self) -> JsSO2State {
        let rotation_rs = self.inner.get_rotation();
        JsSO2State::new(rotation_rs.value)
    }
}
