// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use oxmpl::base::{
    space::{SE3StateSpace as OxmplSE3StateSpace, StateSpace},
    state::{SE3State as OxmplSE3State, SO3State},
};
use rand::rng;
use std::sync::{Arc, Mutex};
use wasm_bindgen::prelude::*;

use crate::base::JsSE3State;

#[wasm_bindgen(js_name = SE3StateSpace)]
pub struct JsSE3StateSpace {
    #[wasm_bindgen(skip)]
    pub inner: Arc<Mutex<OxmplSE3StateSpace>>,
}

#[wasm_bindgen(js_class = SE3StateSpace)]
impl JsSE3StateSpace {
    #[wasm_bindgen(constructor)]
    pub fn new(weight: f64, bounds: Option<Vec<f64>>) -> Result<JsSE3StateSpace, String> {
        let bounds_rs = if let Some(b) = bounds {
            if b.len() != 6 {
                return Err("Bounds must be an array of 6 floats: [x_min, x_max, y_min, y_max, z_min, z_max]".to_string());
            }
            Some(vec![(b[0], b[1]), (b[2], b[3]), (b[4], b[5])])
        } else {
            None
        };

        match OxmplSE3StateSpace::new(weight, bounds_rs) {
            Ok(space) => Ok(JsSE3StateSpace {
                inner: Arc::new(Mutex::new(space)),
            }),
            Err(e) => Err(format!("{e:?}")),
        }
    }

    #[wasm_bindgen(js_name = sample)]
    pub fn sample(&self) -> Result<JsSE3State, String> {
        let mut rng = rng();
        match self.inner.lock().unwrap().sample_uniform(&mut rng) {
            Ok(state) => Ok(JsSE3State {
                inner: Arc::new(state),
            }),
            Err(e) => Err(format!("{e:?}")),
        }
    }

    #[wasm_bindgen(js_name = distance)]
    pub fn distance(&self, state1: &JsSE3State, state2: &JsSE3State) -> f64 {
        self.inner
            .lock()
            .unwrap()
            .distance(&state1.inner, &state2.inner)
    }

    #[wasm_bindgen(js_name = satisfiesBounds)]
    pub fn satisfies_bounds(&self, state: &JsSE3State) -> bool {
        self.inner.lock().unwrap().satisfies_bounds(&state.inner)
    }

    #[wasm_bindgen(js_name = enforceBounds)]
    pub fn enforce_bounds(&self, state: &JsSE3State) -> JsSE3State {
        let mut new_state = (*state.inner).clone();
        self.inner.lock().unwrap().enforce_bounds(&mut new_state);
        JsSE3State {
            inner: Arc::new(new_state),
        }
    }

    #[wasm_bindgen(js_name = interpolate)]
    pub fn interpolate(&self, from: &JsSE3State, to: &JsSE3State, t: f64) -> JsSE3State {
        let mut result_state = OxmplSE3State::new(0.0, 0.0, 0.0, SO3State::identity());
        self.inner
            .lock()
            .unwrap()
            .interpolate(&from.inner, &to.inner, t, &mut result_state);
        JsSE3State {
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
