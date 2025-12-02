// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use std::sync::Arc;

use oxmpl::base::state::SO2State as OxmplSO2State;
use wasm_bindgen::prelude::*;

#[wasm_bindgen(js_name = SO2State)]
pub struct JsSO2State {
    #[wasm_bindgen(skip)]
    pub inner: Arc<OxmplSO2State>,
}

#[wasm_bindgen(js_class = SO2State)]
impl JsSO2State {
    #[wasm_bindgen(constructor)]
    pub fn new(value: f64) -> Self {
        let state = OxmplSO2State::new(value);
        Self {
            inner: Arc::new(state),
        }
    }

    #[wasm_bindgen(getter)]
    pub fn value(&self) -> f64 {
        self.inner.value
    }

    #[wasm_bindgen(js_name = normalise)]
    pub fn normalise(&mut self) -> Self {
        let normalised_value = self.inner.as_ref().clone().normalise().value;
        Self {
            inner: Arc::new(OxmplSO2State::new(normalised_value)),
        }
    }
}
