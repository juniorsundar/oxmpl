// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use std::sync::{Arc, Mutex};

use oxmpl::base::space::{AnyStateSpace, CompoundStateSpace, StateSpace};
use rand::rng;
use wasm_bindgen::prelude::*;

use crate::base::{
    JsCompoundState, JsRealVectorStateSpace, JsSE2StateSpace, JsSE3StateSpace, JsSO2StateSpace,
    JsSO3StateSpace,
};

#[wasm_bindgen(js_name = CompoundStateSpace)]
pub struct JsCompoundStateSpace {
    #[wasm_bindgen(skip)]
    pub inner: Arc<Mutex<CompoundStateSpace>>,
}

#[wasm_bindgen(js_class = CompoundStateSpace)]
impl JsCompoundStateSpace {
    #[wasm_bindgen(js_name = sample)]
    pub fn sample(&self) -> Result<JsCompoundState, String> {
        let mut rng = rng();
        match self.inner.lock().unwrap().sample_uniform(&mut rng) {
            Ok(state) => Ok(JsCompoundState {
                inner: Arc::new(state),
            }),
            Err(e) => Err(e.to_string()),
        }
    }

    #[wasm_bindgen(js_name = distance)]
    pub fn distance(&self, state1: &JsCompoundState, state2: &JsCompoundState) -> f64 {
        self.inner
            .lock()
            .unwrap()
            .distance(&state1.inner, &state2.inner)
    }

    #[wasm_bindgen(js_name = satisfiesBounds)]
    pub fn satisfies_bounds(&self, state: &JsCompoundState) -> bool {
        self.inner.lock().unwrap().satisfies_bounds(&state.inner)
    }

    #[wasm_bindgen(js_name = enforceBounds)]
    pub fn enforce_bounds(&self, state: &JsCompoundState) -> JsCompoundState {
        let mut new_state = (*state.inner).clone();
        self.inner.lock().unwrap().enforce_bounds(&mut new_state);
        JsCompoundState {
            inner: Arc::new(new_state),
        }
    }

    #[wasm_bindgen(js_name = interpolate)]
    pub fn interpolate(
        &self,
        from: &JsCompoundState,
        to: &JsCompoundState,
        t: f64,
    ) -> JsCompoundState {
        let mut result_state = (*from.inner).clone();
        self.inner
            .lock()
            .unwrap()
            .interpolate(&from.inner, &to.inner, t, &mut result_state);
        JsCompoundState {
            inner: Arc::new(result_state),
        }
    }

    #[wasm_bindgen(js_name = getLongestValidSegmentLength)]
    pub fn get_longest_valid_segment_length(&self) -> f64 {
        self.inner
            .lock()
            .unwrap()
            .get_longest_valid_segment_length()
    }
}

#[wasm_bindgen(js_name = CompoundStateSpaceBuilder)]
pub struct JsCompoundStateSpaceBuilder {
    subspaces: Vec<Box<dyn AnyStateSpace>>,
    weights: Vec<f64>,
}

impl Default for JsCompoundStateSpaceBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[wasm_bindgen(js_class = CompoundStateSpaceBuilder)]
impl JsCompoundStateSpaceBuilder {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self {
            subspaces: Vec::new(),
            weights: Vec::new(),
        }
    }

    #[wasm_bindgen(js_name = addRealVectorStateSpace)]
    pub fn add_real_vector_state_space(&mut self, space: &JsRealVectorStateSpace, weight: f64) {
        self.subspaces
            .push(Box::new(space.inner.lock().unwrap().clone()));
        self.weights.push(weight);
    }

    #[wasm_bindgen(js_name = addSO2StateSpace)]
    pub fn add_so2_state_space(&mut self, space: &JsSO2StateSpace, weight: f64) {
        self.subspaces
            .push(Box::new(space.inner.lock().unwrap().clone()));
        self.weights.push(weight);
    }

    #[wasm_bindgen(js_name = addSO3StateSpace)]
    pub fn add_so3_state_space(&mut self, space: &JsSO3StateSpace, weight: f64) {
        self.subspaces
            .push(Box::new(space.inner.lock().unwrap().clone()));
        self.weights.push(weight);
    }

    #[wasm_bindgen(js_name = addSE2StateSpace)]
    pub fn add_se2_state_space(&mut self, space: &JsSE2StateSpace, weight: f64) {
        self.subspaces
            .push(Box::new(space.inner.lock().unwrap().clone()));
        self.weights.push(weight);
    }

    #[wasm_bindgen(js_name = addSE3StateSpace)]
    pub fn add_se3_state_space(&mut self, space: &JsSE3StateSpace, weight: f64) {
        self.subspaces
            .push(Box::new(space.inner.lock().unwrap().clone()));
        self.weights.push(weight);
    }

    #[wasm_bindgen(js_name = addCompoundStateSpace)]
    pub fn add_compound_state_space(&mut self, space: &JsCompoundStateSpace, weight: f64) {
        self.subspaces
            .push(Box::new(space.inner.lock().unwrap().clone()));
        self.weights.push(weight);
    }

    pub fn build(self) -> JsCompoundStateSpace {
        let space = CompoundStateSpace::new(self.subspaces, self.weights);
        JsCompoundStateSpace {
            inner: Arc::new(Mutex::new(space)),
        }
    }
}
