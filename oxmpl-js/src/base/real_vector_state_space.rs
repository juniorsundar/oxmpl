// Copyright (c) 2025 Ross Gardiner, Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use oxmpl::base::{
    space::{RealVectorStateSpace, StateSpace},
    state::RealVectorState,
};
use rand::rng;
use std::sync::Arc;
use wasm_bindgen::prelude::*;

#[wasm_bindgen(js_name = RealVectorStateSpace)]
pub struct JsRealVectorStateSpace {
    #[wasm_bindgen(skip)]
    pub inner: Arc<RealVectorStateSpace>,
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

        match RealVectorStateSpace::new(dimension, bounds_vec) {
            Ok(space) => Ok(Self {
                inner: Arc::new(space),
            }),
            Err(e) => Err(e.to_string()),
        }
    }

    pub fn sample(&self) -> Result<Vec<f64>, String> {
        let mut rng = rng();
        match self.inner.sample_uniform(&mut rng) {
            Ok(state) => Ok(state.values),
            Err(e) => Err(e.to_string()),
        }
    }

    pub fn distance(&self, state1: Vec<f64>, state2: Vec<f64>) -> f64 {
        let s1 = RealVectorState::new(state1);
        let s2 = RealVectorState::new(state2);
        self.inner.distance(&s1, &s2)
    }

    #[wasm_bindgen(js_name = getDimension)]
    pub fn get_dimension(&self) -> usize {
        self.inner.dimension
    }
}
