// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use oxmpl::base::{
    space::{SO3StateSpace as OxmplSO3StateSpace, StateSpace},
    state::SO3State as OxmplSO3State,
};
use rand::rng;
use std::sync::{Arc, Mutex};
use wasm_bindgen::prelude::*;

use crate::base::JsSO3State;

#[wasm_bindgen(js_name = SO3StateSpace)]
pub struct JsSO3StateSpace {
    #[wasm_bindgen(skip)]
    pub inner: Arc<Mutex<OxmplSO3StateSpace>>,
}

#[wasm_bindgen(js_class = SO3StateSpace)]
impl JsSO3StateSpace {
    #[wasm_bindgen(constructor)]
    pub fn new(
        center: Option<JsSO3State>,
        max_angle: Option<f64>,
    ) -> Result<JsSO3StateSpace, String> {
        let bounds = match (center, max_angle) {
            (Some(c), Some(a)) => Some((
                OxmplSO3State::new(c.inner.x, c.inner.y, c.inner.z, c.inner.w),
                a,
            )),
            _ => None,
        };

        match OxmplSO3StateSpace::new(bounds) {
            Ok(space) => Ok(JsSO3StateSpace {
                inner: Arc::new(Mutex::new(space)),
            }),
            Err(e) => Err(format!("{:?}", e)),
        }
    }

    #[wasm_bindgen(js_name = sample)]
    pub fn sample(&self) -> Result<JsSO3State, String> {
        let mut rng = rng();
        match self.inner.lock().unwrap().sample_uniform(&mut rng) {
            Ok(state) => Ok(JsSO3State::new(state.x, state.y, state.z, state.w)),
            Err(e) => Err(e.to_string()),
        }
    }

    #[wasm_bindgen(js_name = distance)]
    pub fn distance(&self, state1: &JsSO3State, state2: &JsSO3State) -> f64 {
        self.inner
            .lock()
            .unwrap()
            .distance(&state1.inner, &state2.inner)
    }

    #[wasm_bindgen(js_name = satisfiesBounds)]
    pub fn satisfies_bounds(&self, state: &JsSO3State) -> bool {
        self.inner.lock().unwrap().satisfies_bounds(&state.inner)
    }

    #[wasm_bindgen(js_name = enforceBounds)]
    pub fn enforce_bounds(&self, state: &JsSO3State) -> JsSO3State {
        let mut new_state = (*state.inner).clone();
        self.inner.lock().unwrap().enforce_bounds(&mut new_state);
        JsSO3State {
            inner: Arc::new(new_state),
        }
    }

    #[wasm_bindgen(js_name = interpolate)]
    pub fn interpolate(&self, from: &JsSO3State, to: &JsSO3State, t: f64) -> JsSO3State {
        let mut result_state = OxmplSO3State::identity();
        self.inner
            .lock()
            .unwrap()
            .interpolate(&from.inner, &to.inner, t, &mut result_state);
        JsSO3State {
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
