// Copyright (c) 2025 Ross Gardiner, Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use oxmpl::base::{
    space::{RealVectorStateSpace as OxmplRealVectorStateSpace, StateSpace},
    state::RealVectorState,
};
use rand::rng;
use std::sync::{Arc, Mutex};
use wasm_bindgen::prelude::*;

use crate::base::JsRealVectorState;

#[wasm_bindgen(js_name = RealVectorStateSpace)]
pub struct JsRealVectorStateSpace {
    #[wasm_bindgen(skip)]
    pub inner: Arc<Mutex<OxmplRealVectorStateSpace>>,
}

#[wasm_bindgen(js_class = RealVectorStateSpace)]
impl JsRealVectorStateSpace {
    #[wasm_bindgen(constructor)]
    pub fn new(
        dimension: usize,
        bounds: Option<Vec<f64>>,
    ) -> Result<JsRealVectorStateSpace, String> {
        let bounds_vec = if let Some(b) = bounds {
            if b.len() != dimension * 2 {
                return Err(format!(
                    "Bounds array must have {} elements (2 per dimension)",
                    dimension * 2
                ));
            }
            let mut bounds_tuples = Vec::new();
            for i in 0..dimension {
                bounds_tuples.push((b[i * 2], b[i * 2 + 1]));
            }
            Some(bounds_tuples)
        } else {
            None
        };

        match OxmplRealVectorStateSpace::new(dimension, bounds_vec) {
            Ok(space) => Ok(Self {
                inner: Arc::new(Mutex::new(space)),
            }),
            Err(e) => Err(e.to_string()),
        }
    }

    #[wasm_bindgen(js_name = sample)]
    pub fn sample(&self) -> Result<JsRealVectorState, String> {
        let mut rng = rng();
        match self.inner.lock().unwrap().sample_uniform(&mut rng) {
            Ok(state) => Ok(JsRealVectorState::new(state.values)),
            Err(e) => Err(e.to_string()),
        }
    }

    #[wasm_bindgen(js_name = distance)]
    pub fn distance(&self, state1: &JsRealVectorState, state2: &JsRealVectorState) -> f64 {
        let s1 = RealVectorState::new(state1.inner.values.clone());
        let s2 = RealVectorState::new(state2.inner.values.clone());
        self.inner.lock().unwrap().distance(&s1, &s2)
    }

    #[wasm_bindgen(js_name = getDimension)]
    pub fn get_dimension(&self) -> usize {
        self.inner.lock().unwrap().dimension
    }

    #[wasm_bindgen(js_name = setLongestValidLineSegmentFraction)]
    pub fn set_longest_valid_segment_fraction(&mut self, fraction: f64) {
        self.inner
            .lock()
            .unwrap()
            .set_longest_valid_segment_fraction(fraction);
    }
}
