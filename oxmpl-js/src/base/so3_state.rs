// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use std::sync::Arc;

use oxmpl::base::state::SO3State as OxmplSO3State;
use wasm_bindgen::prelude::*;

#[wasm_bindgen(js_name = SO3State)]
pub struct JsSO3State {
    #[wasm_bindgen(skip)]
    pub inner: Arc<OxmplSO3State>,
}

#[wasm_bindgen(js_class = SO3State)]
impl JsSO3State {
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64, z: f64, w: f64) -> Self {
        let state = OxmplSO3State::new(x, y, z, w);
        Self {
            inner: Arc::new(state),
        }
    }

    #[wasm_bindgen(getter)]
    pub fn x(&self) -> f64 {
        self.inner.x
    }

    #[wasm_bindgen(getter)]
    pub fn y(&self) -> f64 {
        self.inner.y
    }

    #[wasm_bindgen(getter)]
    pub fn z(&self) -> f64 {
        self.inner.z
    }

    #[wasm_bindgen(getter)]
    pub fn w(&self) -> f64 {
        self.inner.w
    }

    #[wasm_bindgen(js_name = normalise)]
    pub fn normalise(&mut self) -> Result<JsSO3State, String> {
        let normalise_so3_result = self.inner.as_ref().clone().normalise();
        match normalise_so3_result {
            Ok(normalised_so3) => Ok(Self {
                inner: Arc::new(OxmplSO3State::new(
                    normalised_so3.x,
                    normalised_so3.y,
                    normalised_so3.z,
                    normalised_so3.w,
                )),
            }),
            Err(e) => Err(e.to_string()),
        }
    }

    #[wasm_bindgen(js_name = identity)]
    pub fn identity() -> Self {
        Self {
            inner: Arc::new(OxmplSO3State::identity()),
        }
    }
}
