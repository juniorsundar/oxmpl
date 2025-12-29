// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use std::{any::Any, sync::Arc};

use oxmpl::base::state::{
    CompoundState, RealVectorState, SE2State, SE3State, SO2State, SO3State, State,
};
use wasm_bindgen::prelude::*;

use crate::base::{JsRealVectorState, JsSE2State, JsSE3State, JsSO2State, JsSO3State};

#[wasm_bindgen(js_name = CompoundState)]
pub struct JsCompoundState {
    #[wasm_bindgen(skip)]
    pub inner: Arc<CompoundState>,
}

#[wasm_bindgen(js_class = CompoundState)]
impl JsCompoundState {
    #[wasm_bindgen(getter = componentCount)]
    pub fn component_count(&self) -> usize {
        self.inner.components.len()
    }

    #[wasm_bindgen(js_name = getComponent)]
    pub fn get_component(&self, index: usize) -> Result<JsValue, String> {
        if index >= self.inner.components.len() {
            return Err("Index out of bounds".to_string());
        }

        let component = &self.inner.components[index];
        let component_ref: &dyn State = component.as_ref();
        let any_ref = component_ref as &dyn Any;

        if let Some(s) = any_ref.downcast_ref::<RealVectorState>() {
            return Ok(JsValue::from(JsRealVectorState {
                inner: Arc::new(s.clone()),
            }));
        }
        if let Some(s) = any_ref.downcast_ref::<SO2State>() {
            return Ok(JsValue::from(JsSO2State::new(s.value)));
        }
        if let Some(s) = any_ref.downcast_ref::<SO3State>() {
            return Ok(JsValue::from(JsSO3State::new(s.x, s.y, s.z, s.w)));
        }
        if let Some(s) = any_ref.downcast_ref::<SE2State>() {
            return Ok(JsValue::from(JsSE2State::new(
                s.get_x(),
                s.get_y(),
                s.get_yaw(),
            )));
        }
        if let Some(s) = any_ref.downcast_ref::<SE3State>() {
            let rotation = JsSO3State::new(
                s.get_rotation().x,
                s.get_rotation().y,
                s.get_rotation().z,
                s.get_rotation().w,
            );
            return Ok(JsValue::from(JsSE3State::new(
                s.get_x(),
                s.get_y(),
                s.get_z(),
                rotation,
            )));
        }
        if let Some(s) = any_ref.downcast_ref::<CompoundState>() {
            return Ok(JsValue::from(JsCompoundState {
                inner: Arc::new(s.clone()),
            }));
        }

        Err("Unknown component state type".to_string())
    }
}

/// A builder for creating `CompoundState` instances from JavaScript.
///
/// The builder pattern is used here instead of Runtime Type Inferencing (downcasting) from generic
/// JavaScript objects because the latter required `unsafe` blocks or complex trait machinery that
/// is not well-supported by `wasm-bindgen`.
#[wasm_bindgen(js_name = CompoundStateBuilder)]
pub struct JsCompoundStateBuilder {
    components: Vec<Box<dyn State>>,
}

impl Default for JsCompoundStateBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[wasm_bindgen(js_class = CompoundStateBuilder)]
impl JsCompoundStateBuilder {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self {
            components: Vec::new(),
        }
    }

    #[wasm_bindgen(js_name = addRealVectorState)]
    pub fn add_real_vector_state(&mut self, state: &JsRealVectorState) {
        self.components.push(Box::new((*state.inner).clone()));
    }

    #[wasm_bindgen(js_name = addSO2State)]
    pub fn add_so2_state(&mut self, state: &JsSO2State) {
        self.components.push(Box::new((*state.inner).clone()));
    }

    #[wasm_bindgen(js_name = addSO3State)]
    pub fn add_so3_state(&mut self, state: &JsSO3State) {
        self.components.push(Box::new((*state.inner).clone()));
    }

    #[wasm_bindgen(js_name = addSE2State)]
    pub fn add_se2_state(&mut self, state: &JsSE2State) {
        self.components.push(Box::new((*state.inner).clone()));
    }

    #[wasm_bindgen(js_name = addSE3State)]
    pub fn add_se3_state(&mut self, state: &JsSE3State) {
        self.components.push(Box::new((*state.inner).clone()));
    }

    #[wasm_bindgen(js_name = addCompoundState)]
    pub fn add_compound_state(&mut self, state: &JsCompoundState) {
        self.components.push(Box::new((*state.inner).clone()));
    }

    pub fn build(self) -> JsCompoundState {
        JsCompoundState {
            inner: Arc::new(CompoundState::new(self.components)),
        }
    }
}
