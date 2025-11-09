// Copyright (c) 2025 Ross Gardiner, Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use js_sys::Float64Array;
use oxmpl::base::state::RealVectorState;
use wasm_bindgen::prelude::*;

// Helper functions for converting between Rust states and JavaScript arrays
pub fn state_to_js_array(state: &RealVectorState) -> Float64Array {
    let array = Float64Array::new_with_length(state.values.len() as u32);
    for (i, &val) in state.values.iter().enumerate() {
        array.set_index(i as u32, val);
    }
    array
}

pub fn js_array_to_state(array: &Float64Array) -> RealVectorState {
    let mut values = Vec::new();
    for i in 0..array.length() {
        values.push(array.get_index(i));
    }
    RealVectorState::new(values)
}

// Set panic hook to get better error messages
#[wasm_bindgen(start)]
pub fn set_panic_hook() {
    console_error_panic_hook::set_once();
}

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = console)]
    fn log(s: &str);
}
