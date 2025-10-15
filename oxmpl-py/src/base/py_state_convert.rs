// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::{
    compound_state::PyCompoundState, real_vector_state::PyRealVectorState, se2_state::PySE2State,
    se3_state::PySE3State, so2_state::PySO2State, so3_state::PySO3State,
};
use oxmpl::base::state::{
    CompoundState as OxmplCompoundState, RealVectorState as OxmplRealVectorState,
    SE2State as OxmplSE2State, SE3State as OxmplSE3State, SO2State as OxmplSO2State,
    SO3State as OxmplSO3State,
};
use pyo3::prelude::*;
use std::{rc::Rc, sync::Arc};

/// A trait to handle conversions between a core Rust state and its PyO3 wrapper.
pub trait PyStateConvert: Clone + Send + Sync + 'static {
    type Wrapper: for<'a> FromPyObject<'a> + for<'a> IntoPyObject<'a>;

    fn to_py_wrapper(&self) -> Self::Wrapper;

    fn from_py_wrapper(wrapper: Self::Wrapper) -> Self;
}

impl PyStateConvert for OxmplRealVectorState {
    type Wrapper = PyRealVectorState;

    fn to_py_wrapper(&self) -> Self::Wrapper {
        PyRealVectorState(Arc::new(self.clone()))
    }

    fn from_py_wrapper(wrapper: Self::Wrapper) -> Self {
        (*wrapper.0).clone()
    }
}

impl PyStateConvert for OxmplSO2State {
    type Wrapper = PySO2State;

    fn to_py_wrapper(&self) -> Self::Wrapper {
        PySO2State(Arc::new(self.clone()))
    }

    fn from_py_wrapper(wrapper: Self::Wrapper) -> Self {
        (*wrapper.0).clone()
    }
}

impl PyStateConvert for OxmplSO3State {
    type Wrapper = PySO3State;

    fn to_py_wrapper(&self) -> Self::Wrapper {
        PySO3State(Arc::new(self.clone()))
    }

    fn from_py_wrapper(wrapper: Self::Wrapper) -> Self {
        (*wrapper.0).clone()
    }
}

impl PyStateConvert for OxmplCompoundState {
    type Wrapper = PyCompoundState;

    fn to_py_wrapper(&self) -> Self::Wrapper {
        PyCompoundState(Rc::new(self.clone()))
    }

    fn from_py_wrapper(wrapper: Self::Wrapper) -> Self {
        (*wrapper.0).clone()
    }
}

impl PyStateConvert for OxmplSE2State {
    type Wrapper = PySE2State;

    fn to_py_wrapper(&self) -> Self::Wrapper {
        PySE2State(Rc::new(self.clone()))
    }

    fn from_py_wrapper(wrapper: Self::Wrapper) -> Self {
        (*wrapper.0).clone()
    }
}

impl PyStateConvert for OxmplSE3State {
    type Wrapper = PySE3State;

    fn to_py_wrapper(&self) -> Self::Wrapper {
        PySE3State(Rc::new(self.clone()))
    }

    fn from_py_wrapper(wrapper: Self::Wrapper) -> Self {
        (*wrapper.0).clone()
    }
}
