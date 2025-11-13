// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use std::sync::Arc;

use oxmpl::base::state::RealVectorState as OxmplRealVectorState;
use wasm_bindgen::prelude::*;

#[wasm_bindgen(js_name = RealVectorState)]
pub struct JsRealVectorState {
    #[wasm_bindgen(skip)]
    pub inner: Arc<OxmplRealVectorState>,
}

#[wasm_bindgen(js_class = RealVectorState)]
impl JsRealVectorState {
    #[wasm_bindgen(constructor)]
    pub fn new(values: Vec<f64>) -> Self {
        let state = OxmplRealVectorState::new(values);
        Self {
            inner: Arc::new(state),
        }
    }

    #[wasm_bindgen(getter)]
    pub fn values(&self) -> Vec<f64> {
        self.inner.values.clone()
    }
}
