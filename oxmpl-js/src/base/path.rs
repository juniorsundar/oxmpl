// Copyright (c) 2025 Ross Gardiner, Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::js_state_convert::*;
use js_sys::Array;
use oxmpl::base::{
    planner::Path as OxmplPath,
    state::{CompoundState, RealVectorState, SE2State, SE3State, SO2State, SO3State},
};
use wasm_bindgen::prelude::*;

pub enum PathVariant {
    RealVector(OxmplPath<RealVectorState>),
    SO2(OxmplPath<SO2State>),
    SO3(OxmplPath<SO3State>),
    Compound(OxmplPath<CompoundState>),
    SE2(OxmplPath<SE2State>),
    SE3(OxmplPath<SE3State>),
}

#[wasm_bindgen(js_name = Path)]
pub struct JsPath {
    pub(crate) inner: PathVariant,
}

#[wasm_bindgen(js_class = Path)]
impl JsPath {
    #[wasm_bindgen(js_name = getStates)]
    pub fn get_states(&self) -> Array {
        match &self.inner {
            PathVariant::RealVector(path) => path.0.iter().map(|s| s.to_js_value()).collect(),
            PathVariant::SO2(path) => path.0.iter().map(|s| s.to_js_value()).collect(),
            PathVariant::SO3(path) => path.0.iter().map(|s| s.to_js_value()).collect(),
            PathVariant::Compound(path) => path.0.iter().map(|s| s.to_js_value()).collect(),
            PathVariant::SE2(path) => path.0.iter().map(|s| s.to_js_value()).collect(),
            PathVariant::SE3(path) => path.0.iter().map(|s| s.to_js_value()).collect(),
        }
    }

    #[wasm_bindgen(js_name = getLength)]
    pub fn get_length(&self) -> usize {
        match &self.inner {
            PathVariant::RealVector(path) => path.0.len(),
            PathVariant::SO2(path) => path.0.len(),
            PathVariant::SO3(path) => path.0.len(),
            PathVariant::Compound(path) => path.0.len(),
            PathVariant::SE2(path) => path.0.len(),
            PathVariant::SE3(path) => path.0.len(),
        }
    }
}

impl From<OxmplPath<RealVectorState>> for JsPath {
    fn from(path: OxmplPath<RealVectorState>) -> Self {
        Self {
            inner: PathVariant::RealVector(path),
        }
    }
}

impl From<OxmplPath<SO2State>> for JsPath {
    fn from(path: OxmplPath<SO2State>) -> Self {
        Self {
            inner: PathVariant::SO2(path),
        }
    }
}

impl From<OxmplPath<SO3State>> for JsPath {
    fn from(path: OxmplPath<SO3State>) -> Self {
        Self {
            inner: PathVariant::SO3(path),
        }
    }
}

impl From<OxmplPath<CompoundState>> for JsPath {
    fn from(path: OxmplPath<CompoundState>) -> Self {
        Self {
            inner: PathVariant::Compound(path),
        }
    }
}

impl From<OxmplPath<SE2State>> for JsPath {
    fn from(path: OxmplPath<SE2State>) -> Self {
        Self {
            inner: PathVariant::SE2(path),
        }
    }
}

impl From<OxmplPath<SE3State>> for JsPath {
    fn from(path: OxmplPath<SE3State>) -> Self {
        Self {
            inner: PathVariant::SE3(path),
        }
    }
}
