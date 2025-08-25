// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::{real_vector_state::PyRealVectorState, so2_state::PySO2State};
use oxmpl::base::state::{RealVectorState as OxmplRealVectorState, SO2State as OxmplSO2State};
use pyo3::prelude::*;
use std::sync::Arc;

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
