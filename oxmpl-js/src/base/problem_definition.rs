// Copyright (c) 2025 Ross Gardiner, Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::{
    compound_state::JsCompoundState, compound_state_space::JsCompoundStateSpace, goal::JsGoal,
    real_vector_state::JsRealVectorState, real_vector_state_space::JsRealVectorStateSpace,
    se2_state::JsSE2State, se2_state_space::JsSE2StateSpace, se3_state::JsSE3State,
    se3_state_space::JsSE3StateSpace, so2_state::JsSO2State, so2_state_space::JsSO2StateSpace,
    so3_state::JsSO3State, so3_state_space::JsSO3StateSpace,
};
use oxmpl::base::{
    problem_definition::ProblemDefinition,
    space::{
        CompoundStateSpace, RealVectorStateSpace, SE2StateSpace, SE3StateSpace, SO2StateSpace,
        SO3StateSpace,
    },
    state::{CompoundState, RealVectorState, SE2State, SE3State, SO2State, SO3State},
};
use std::sync::Arc;
use wasm_bindgen::prelude::*;

pub enum ProblemDefinitionVariant {
    RealVector(Arc<ProblemDefinition<RealVectorState, RealVectorStateSpace, JsGoal>>),
    SO2(Arc<ProblemDefinition<SO2State, SO2StateSpace, JsGoal>>),
    SO3(Arc<ProblemDefinition<SO3State, SO3StateSpace, JsGoal>>),
    SE2(Arc<ProblemDefinition<SE2State, SE2StateSpace, JsGoal>>),
    SE3(Arc<ProblemDefinition<SE3State, SE3StateSpace, JsGoal>>),
    Compound(Arc<ProblemDefinition<CompoundState, CompoundStateSpace, JsGoal>>),
}

#[wasm_bindgen(js_name = ProblemDefinition)]
pub struct JsProblemDefinition {
    pub(crate) inner: ProblemDefinitionVariant,
}

#[wasm_bindgen(js_class = ProblemDefinition)]
impl JsProblemDefinition {
    #[wasm_bindgen(js_name = fromRealVectorState)]
    pub fn from_real_vector_state(
        space: &JsRealVectorStateSpace,
        start: &JsRealVectorState,
        goal: &JsGoal,
    ) -> Self {
        let pd = ProblemDefinition {
            space: Arc::new(space.inner.lock().unwrap().clone()),
            start_states: vec![(*start.inner).clone()],
            goal: Arc::new(goal.clone()),
        };
        Self {
            inner: ProblemDefinitionVariant::RealVector(Arc::new(pd)),
        }
    }

    #[wasm_bindgen(js_name = fromSO2State)]
    pub fn from_so2_state(space: &JsSO2StateSpace, start: &JsSO2State, goal: &JsGoal) -> Self {
        let pd = ProblemDefinition {
            space: Arc::new(space.inner.lock().unwrap().clone()),
            start_states: vec![(*start.inner).clone()],
            goal: Arc::new(goal.clone()),
        };
        Self {
            inner: ProblemDefinitionVariant::SO2(Arc::new(pd)),
        }
    }

    #[wasm_bindgen(js_name = fromSO3State)]
    pub fn from_so3_state(space: &JsSO3StateSpace, start: &JsSO3State, goal: &JsGoal) -> Self {
        let pd = ProblemDefinition {
            space: Arc::new(space.inner.lock().unwrap().clone()),
            start_states: vec![(*start.inner).clone()],
            goal: Arc::new(goal.clone()),
        };
        Self {
            inner: ProblemDefinitionVariant::SO3(Arc::new(pd)),
        }
    }

    #[wasm_bindgen(js_name = fromSE2State)]
    pub fn from_se2_state(space: &JsSE2StateSpace, start: &JsSE2State, goal: &JsGoal) -> Self {
        let pd = ProblemDefinition {
            space: Arc::new(space.inner.lock().unwrap().clone()),
            start_states: vec![(*start.inner).clone()],
            goal: Arc::new(goal.clone()),
        };
        Self {
            inner: ProblemDefinitionVariant::SE2(Arc::new(pd)),
        }
    }

    #[wasm_bindgen(js_name = fromSE3State)]
    pub fn from_se3_state(space: &JsSE3StateSpace, start: &JsSE3State, goal: &JsGoal) -> Self {
        let pd = ProblemDefinition {
            space: Arc::new(space.inner.lock().unwrap().clone()),
            start_states: vec![(*start.inner).clone()],
            goal: Arc::new(goal.clone()),
        };
        Self {
            inner: ProblemDefinitionVariant::SE3(Arc::new(pd)),
        }
    }

    #[wasm_bindgen(js_name = fromCompoundState)]
    pub fn from_compound_state(
        space: &JsCompoundStateSpace,
        start: &JsCompoundState,
        goal: &JsGoal,
    ) -> Self {
        let pd = ProblemDefinition {
            space: Arc::new(space.inner.lock().unwrap().clone()),
            start_states: vec![(*start.inner).clone()],
            goal: Arc::new(goal.clone()),
        };
        Self {
            inner: ProblemDefinitionVariant::Compound(Arc::new(pd)),
        }
    }
}
