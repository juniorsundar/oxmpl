// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use std::sync::Arc;

use oxmpl::base::state::SE3State as OxmplSE3State;
use wasm_bindgen::prelude::*;

use crate::base::{JsRealVectorState, JsSO3State};

#[wasm_bindgen(js_name = SE3State)]
pub struct JsSE3State {
    #[wasm_bindgen(skip)]
    pub inner: Arc<OxmplSE3State>,
}

#[wasm_bindgen(js_class = SE3State)]
impl JsSE3State {
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64, z: f64, rotation: JsSO3State) -> Self {
        let state = OxmplSE3State::new(x, y, z, (*rotation.inner).clone());
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
    pub fn z(&self) -> f64 {
        self.inner.get_z()
    }
    #[wasm_bindgen(getter)]
    pub fn translation(&self) -> JsRealVectorState {
        let translation_rs = self.inner.get_translation();
        JsRealVectorState::new(translation_rs.values.clone())
    }
    #[wasm_bindgen(getter)]
    pub fn rotation(&self) -> JsSO3State {
        let rotation_rs = self.inner.get_rotation();
        JsSO3State {
            inner: Arc::new(rotation_rs.clone()),
        }
    }
}
