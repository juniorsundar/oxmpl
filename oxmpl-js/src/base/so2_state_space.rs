// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use oxmpl::base::{
    space::{SO2StateSpace as OxmplSO2StateSpace, StateSpace},
    state::SO2State as OxmplSO2State,
};
use rand::rng;
use std::sync::{Arc, Mutex};
use wasm_bindgen::prelude::*;

use crate::base::JsSO2State;

#[wasm_bindgen(js_name = SO2StateSpace)]
pub struct JsSO2StateSpace {
    #[wasm_bindgen(skip)]
    pub inner: Arc<Mutex<OxmplSO2StateSpace>>,
}

#[wasm_bindgen(js_class = SO2StateSpace)]
impl JsSO2StateSpace {
    #[wasm_bindgen(constructor)]
    pub fn new(bounds_option: Option<Box<[f64]>>) -> Result<JsSO2StateSpace, String> {
        let bounds_tuple = if let Some(arr) = bounds_option {
            if arr.len() != 2 {
                return Err(format!(
                    "Invalid bounds: expected an array of length 2, but received length {}.",
                    arr.len()
                ));
            }
            Some((arr[0], arr[1]))
        } else {
            None
        };

        match OxmplSO2StateSpace::new(bounds_tuple) {
            Ok(space) => Ok(Self {
                inner: Arc::new(Mutex::new(space)),
            }),
            Err(e) => Err(e.to_string()),
        }
    }

    #[wasm_bindgen(js_name = sample)]
    pub fn sample(&self) -> Result<JsSO2State, String> {
        let mut rng = rng();
        match self.inner.lock().unwrap().sample_uniform(&mut rng) {
            Ok(state) => Ok(JsSO2State::new(state.value)),
            Err(e) => Err(e.to_string()),
        }
    }

    #[wasm_bindgen(js_name = distance)]
    pub fn distance(&self, state1: &JsSO2State, state2: &JsSO2State) -> f64 {
        self.inner
            .lock()
            .unwrap()
            .distance(&state1.inner, &state2.inner)
    }

    #[wasm_bindgen(js_name = satisfiesBounds)]
    pub fn satisfies_bounds(&self, state: &JsSO2State) -> bool {
        self.inner.lock().unwrap().satisfies_bounds(&state.inner)
    }

    #[wasm_bindgen(js_name = enforceBounds)]
    pub fn enforce_bounds(&self, state: &JsSO2State) -> JsSO2State {
        let mut new_state = (*state.inner).clone();
        self.inner.lock().unwrap().enforce_bounds(&mut new_state);
        JsSO2State {
            inner: Arc::new(new_state),
        }
    }

    #[wasm_bindgen(js_name = interpolate)]
    pub fn interpolate(&self, from: &JsSO2State, to: &JsSO2State, t: f64) -> JsSO2State {
        let mut result_state = OxmplSO2State::new(0.0);
        self.inner
            .lock()
            .unwrap()
            .interpolate(&from.inner, &to.inner, t, &mut result_state);
        JsSO2State {
            inner: Arc::new(result_state),
        }
    }

    #[wasm_bindgen(js_name = getMaximumExtent)]
    pub fn get_maximum_extent(&self) -> f64 {
        self.inner.lock().unwrap().get_maximum_extent()
    }

    #[wasm_bindgen(js_name = getLongestValidSegmentLength)]
    pub fn get_longest_valid_segment_length(&self) -> f64 {
        self.inner
            .lock()
            .unwrap()
            .get_longest_valid_segment_length()
    }

    #[wasm_bindgen(js_name = setLongestValidLineSegmentFraction)]
    pub fn set_longest_valid_segment_fraction(&mut self, fraction: f64) {
        self.inner
            .lock()
            .unwrap()
            .set_longest_valid_segment_fraction(fraction);
    }
}
