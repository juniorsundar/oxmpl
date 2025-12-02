// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use oxmpl::base::{
    space::{SE2StateSpace as OxmplSE2StateSpace, StateSpace},
    state::SE2State as OxmplSE2State,
};
use rand::rng;
use std::sync::{Arc, Mutex};
use wasm_bindgen::prelude::*;

use crate::base::JsSE2State;

#[wasm_bindgen(js_name = SE2StateSpace)]
pub struct JsSE2StateSpace {
    #[wasm_bindgen(skip)]
    pub inner: Arc<Mutex<OxmplSE2StateSpace>>,
}

#[wasm_bindgen(js_class = SE2StateSpace)]
impl JsSE2StateSpace {
    #[wasm_bindgen(constructor)]
    pub fn new(weight: f64, bounds: Option<Vec<f64>>) -> Result<JsSE2StateSpace, String> {
        let bounds_rs = if let Some(b) = bounds {
            if b.len() != 6 {
                return Err("Bounds must be an array of 6 floats: [x_min, x_max, y_min, y_max, yaw_min, yaw_max]".to_string());
            }
            Some(vec![(b[0], b[1]), (b[2], b[3]), (b[4], b[5])])
        } else {
            None
        };

        match OxmplSE2StateSpace::new(weight, bounds_rs) {
            Ok(space) => Ok(JsSE2StateSpace {
                inner: Arc::new(Mutex::new(space)),
            }),
            Err(e) => Err(format!("{e:?}")),
        }
    }

    #[wasm_bindgen(js_name = sample)]
    pub fn sample(&self) -> Result<JsSE2State, String> {
        let mut rng = rng();
        match self.inner.lock().unwrap().sample_uniform(&mut rng) {
            Ok(state) => Ok(JsSE2State {
                inner: Arc::new(state),
            }),
            Err(e) => Err(format!("{e:?}")),
        }
    }

    #[wasm_bindgen(js_name = distance)]
    pub fn distance(&self, state1: &JsSE2State, state2: &JsSE2State) -> f64 {
        self.inner
            .lock()
            .unwrap()
            .distance(&state1.inner, &state2.inner)
    }

    #[wasm_bindgen(js_name = satisfiesBounds)]
    pub fn satisfies_bounds(&self, state: &JsSE2State) -> bool {
        self.inner.lock().unwrap().satisfies_bounds(&state.inner)
    }

    #[wasm_bindgen(js_name = enforceBounds)]
    pub fn enforce_bounds(&self, state: &JsSE2State) -> JsSE2State {
        let mut new_state = (*state.inner).clone();
        self.inner.lock().unwrap().enforce_bounds(&mut new_state);
        JsSE2State {
            inner: Arc::new(new_state),
        }
    }

    #[wasm_bindgen(js_name = interpolate)]
    pub fn interpolate(&self, from: &JsSE2State, to: &JsSE2State, t: f64) -> JsSE2State {
        let mut result_state = OxmplSE2State::new(0.0, 0.0, 0.0);
        self.inner
            .lock()
            .unwrap()
            .interpolate(&from.inner, &to.inner, t, &mut result_state);
        JsSE2State {
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
