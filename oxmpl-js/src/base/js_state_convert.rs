// Copyright (c) 2025 Ross Gardiner, Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use std::sync::Arc;

use js_sys::Float64Array;
use oxmpl::base::state::{
    CompoundState, RealVectorState, SE2State, SE3State, SO2State, SO3State, State,
};
use wasm_bindgen::prelude::*;

use crate::base::{
    JsCompoundState, JsRealVectorState, JsSE2State, JsSE3State, JsSO2State, JsSO3State,
};

/// Trait for converting between Rust states and their JavaScript wrappers.
pub trait JsStateConvert: Sized {
    /// Convert the Rust state to a JavaScript wrapper object.
    fn to_js_value(&self) -> JsValue;
    /// Convert a JavaScript wrapper object back to the Rust state.
    fn from_js_value(val: JsValue) -> Result<Self, String>;
}

impl JsStateConvert for RealVectorState {
    fn to_js_value(&self) -> JsValue {
        JsValue::from(JsRealVectorState {
            inner: Arc::new(self.clone()),
        })
    }
    fn from_js_value(val: JsValue) -> Result<Self, String> {
        if let Ok(values_val) = js_sys::Reflect::get(&val, &JsValue::from_str("values")) {
            if let Ok(values) = serde_wasm_bindgen::from_value::<Vec<f64>>(values_val) {
                return Ok(RealVectorState::new(values));
            }
        }
        Err("Expected RealVectorState or object with 'values' property".to_string())
    }
}

impl JsStateConvert for SO2State {
    fn to_js_value(&self) -> JsValue {
        JsValue::from(JsSO2State {
            inner: Arc::new(self.clone()),
        })
    }
    fn from_js_value(val: JsValue) -> Result<Self, String> {
        if let Ok(v_val) = js_sys::Reflect::get(&val, &JsValue::from_str("value")) {
            if let Ok(v) = v_val.as_f64().ok_or("Expected number for SO2 value") {
                return Ok(SO2State::new(v));
            }
        }
        if let Some(v) = val.as_f64() {
            return Ok(SO2State::new(v));
        }
        Err("Expected SO2State, object with 'value' property, or number".to_string())
    }
}

impl JsStateConvert for SO3State {
    fn to_js_value(&self) -> JsValue {
        JsValue::from(JsSO3State {
            inner: Arc::new(self.clone()),
        })
    }
    fn from_js_value(val: JsValue) -> Result<Self, String> {
        let x = js_sys::Reflect::get(&val, &JsValue::from_str("x"))
            .ok()
            .and_then(|v| v.as_f64());
        let y = js_sys::Reflect::get(&val, &JsValue::from_str("y"))
            .ok()
            .and_then(|v| v.as_f64());
        let z = js_sys::Reflect::get(&val, &JsValue::from_str("z"))
            .ok()
            .and_then(|v| v.as_f64());
        let w = js_sys::Reflect::get(&val, &JsValue::from_str("w"))
            .ok()
            .and_then(|v| v.as_f64());

        if let (Some(x), Some(y), Some(z), Some(w)) = (x, y, z, w) {
            Ok(SO3State::new(x, y, z, w))
        } else {
            Err("Expected SO3State or object with x, y, z, w properties".to_string())
        }
    }
}

impl JsStateConvert for SE2State {
    fn to_js_value(&self) -> JsValue {
        JsValue::from(JsSE2State {
            inner: Arc::new(self.clone()),
        })
    }
    fn from_js_value(val: JsValue) -> Result<Self, String> {
        let x = js_sys::Reflect::get(&val, &JsValue::from_str("x"))
            .ok()
            .and_then(|v| v.as_f64());
        let y = js_sys::Reflect::get(&val, &JsValue::from_str("y"))
            .ok()
            .and_then(|v| v.as_f64());
        let yaw = js_sys::Reflect::get(&val, &JsValue::from_str("yaw"))
            .ok()
            .and_then(|v| v.as_f64());

        if let (Some(x), Some(y), Some(yaw)) = (x, y, yaw) {
            Ok(SE2State::new(x, y, yaw))
        } else {
            Err("Expected SE2State or object with x, y, yaw properties".to_string())
        }
    }
}

impl JsStateConvert for SE3State {
    fn to_js_value(&self) -> JsValue {
        JsValue::from(JsSE3State {
            inner: Arc::new(self.clone()),
        })
    }
    fn from_js_value(val: JsValue) -> Result<Self, String> {
        let x = js_sys::Reflect::get(&val, &JsValue::from_str("x"))
            .ok()
            .and_then(|v| v.as_f64());
        let y = js_sys::Reflect::get(&val, &JsValue::from_str("y"))
            .ok()
            .and_then(|v| v.as_f64());
        let z = js_sys::Reflect::get(&val, &JsValue::from_str("z"))
            .ok()
            .and_then(|v| v.as_f64());
        let rotation_val = js_sys::Reflect::get(&val, &JsValue::from_str("rotation")).ok();

        if let (Some(x), Some(y), Some(z), Some(rot_val)) = (x, y, z, rotation_val) {
            let rotation = SO3State::from_js_value(rot_val)?;
            Ok(SE3State::new(x, y, z, rotation))
        } else {
            Err("Expected SE3State or object with x, y, z, and rotation properties".to_string())
        }
    }
}

impl JsStateConvert for CompoundState {
    fn to_js_value(&self) -> JsValue {
        JsValue::from(JsCompoundState {
            inner: Arc::new(self.clone()),
        })
    }
    fn from_js_value(_val: JsValue) -> Result<Self, String> {
        Err("CompoundState from_js_value not implemented".to_string())
    }
}

// Helper functions for converting between Rust states and JavaScript arrays

pub fn real_vector_state_to_js_array(state: &RealVectorState) -> Float64Array {
    let array = Float64Array::new_with_length(state.values.len() as u32);
    for (i, &val) in state.values.iter().enumerate() {
        array.set_index(i as u32, val);
    }
    array
}

pub fn js_array_to_real_vector_state(array: &Float64Array) -> RealVectorState {
    let mut values = Vec::new();
    for i in 0..array.length() {
        values.push(array.get_index(i));
    }
    RealVectorState::new(values)
}

pub fn so2_state_to_js_array(state: &SO2State) -> Float64Array {
    let array = Float64Array::new_with_length(1);
    array.set_index(0, state.value);
    array
}

pub fn js_array_to_so2_state(array: &Float64Array) -> SO2State {
    let val = if array.length() > 0 {
        array.get_index(0)
    } else {
        0.0
    };
    SO2State::new(val)
}

pub fn se2_state_to_js_array(state: &SE2State) -> Float64Array {
    let array = Float64Array::new_with_length(3);
    array.set_index(0, state.get_x());
    array.set_index(1, state.get_y());
    array.set_index(2, state.get_yaw());
    array
}

pub fn js_array_to_se2_state(array: &Float64Array) -> SE2State {
    let x = if array.length() > 0 {
        array.get_index(0)
    } else {
        0.0
    };
    let y = if array.length() > 1 {
        array.get_index(1)
    } else {
        0.0
    };
    let yaw = if array.length() > 2 {
        array.get_index(2)
    } else {
        0.0
    };
    SE2State::new(x, y, yaw)
}

pub fn so3_state_to_js_array(state: &SO3State) -> Float64Array {
    let array = Float64Array::new_with_length(4);
    array.set_index(0, state.x);
    array.set_index(1, state.y);
    array.set_index(2, state.z);
    array.set_index(3, state.w);
    array
}

pub fn js_array_to_so3_state(array: &Float64Array) -> SO3State {
    let x = if array.length() > 0 {
        array.get_index(0)
    } else {
        0.0
    };
    let y = if array.length() > 1 {
        array.get_index(1)
    } else {
        0.0
    };
    let z = if array.length() > 2 {
        array.get_index(2)
    } else {
        0.0
    };
    let w = if array.length() > 3 {
        array.get_index(3)
    } else {
        1.0
    };
    SO3State::new(x, y, z, w)
}

pub fn se3_state_to_js_array(state: &SE3State) -> Float64Array {
    let array = Float64Array::new_with_length(7);
    array.set_index(0, state.get_x());
    array.set_index(1, state.get_y());
    array.set_index(2, state.get_z());
    array.set_index(3, state.get_rotation().x);
    array.set_index(4, state.get_rotation().y);
    array.set_index(5, state.get_rotation().z);
    array.set_index(6, state.get_rotation().w);
    array
}

pub fn js_array_to_se3_state(array: &Float64Array) -> SE3State {
    let x = if array.length() > 0 {
        array.get_index(0)
    } else {
        0.0
    };
    let y = if array.length() > 1 {
        array.get_index(1)
    } else {
        0.0
    };
    let z = if array.length() > 2 {
        array.get_index(2)
    } else {
        0.0
    };
    let qx = if array.length() > 3 {
        array.get_index(3)
    } else {
        0.0
    };
    let qy = if array.length() > 4 {
        array.get_index(4)
    } else {
        0.0
    };
    let qz = if array.length() > 5 {
        array.get_index(5)
    } else {
        0.0
    };
    let qw = if array.length() > 6 {
        array.get_index(6)
    } else {
        1.0
    };
    SE3State::new(x, y, z, SO3State::new(qx, qy, qz, qw))
}

pub fn compound_state_to_js_array(state: &CompoundState) -> Float64Array {
    let mut values = Vec::new();
    // Helper to recursively flatten
    fn flatten(s: &dyn State, out: &mut Vec<f64>) {
        let any_s = s.as_any();
        if let Some(rv) = any_s.downcast_ref::<RealVectorState>() {
            out.extend_from_slice(&rv.values);
        } else if let Some(so2) = any_s.downcast_ref::<SO2State>() {
            out.push(so2.value);
        } else if let Some(se2) = any_s.downcast_ref::<SE2State>() {
            out.push(se2.get_x());
            out.push(se2.get_y());
            out.push(se2.get_yaw());
        } else if let Some(so3) = any_s.downcast_ref::<SO3State>() {
            out.push(so3.x);
            out.push(so3.y);
            out.push(so3.z);
            out.push(so3.w);
        } else if let Some(se3) = any_s.downcast_ref::<SE3State>() {
            out.push(se3.get_x());
            out.push(se3.get_y());
            out.push(se3.get_z());
            out.push(se3.get_rotation().x);
            out.push(se3.get_rotation().y);
            out.push(se3.get_rotation().z);
            out.push(se3.get_rotation().w);
        } else if let Some(comp) = any_s.downcast_ref::<CompoundState>() {
            for c in &comp.components {
                flatten(c.as_ref(), out);
            }
        }
    }

    for component in &state.components {
        flatten(component.as_ref(), &mut values);
    }

    let array = Float64Array::new_with_length(values.len() as u32);
    for (i, &val) in values.iter().enumerate() {
        array.set_index(i as u32, val);
    }
    array
}

// Re-export old names for compatibility if needed (but we will update goal.rs)
pub fn state_to_js_array(state: &RealVectorState) -> Float64Array {
    real_vector_state_to_js_array(state)
}

pub fn js_array_to_state(array: &Float64Array) -> RealVectorState {
    js_array_to_real_vector_state(array)
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
