// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::{exceptions::PyValueError, prelude::*};
use std::{cell::RefCell, rc::Rc};

use oxmpl::base::space::{
    AnyStateSpace, CompoundStateSpace as OxmplCompoundStateSpace, StateSpace as _,
};

use crate::base::{PyRealVectorStateSpace, PySO2StateSpace, PySO3StateSpace};

use super::compound_state::PyCompoundState;

#[pyclass(name = "CompoundStateSpace", unsendable)]
#[derive(Clone)]
pub struct PyCompoundStateSpace(pub Rc<RefCell<OxmplCompoundStateSpace>>);

#[pymethods]
impl PyCompoundStateSpace {
    #[new]
    #[pyo3(signature = (subspaces, weights))]
    fn new(subspaces: Vec<PyObject>, weights: Vec<f64>) -> PyResult<Self> {
        if subspaces.len() != weights.len() {
            return Err(PyValueError::new_err(format!(
                "Number of subspaces ({}) must match number of weights ({}).",
                subspaces.len(),
                weights.len()
            )));
        }

        let mut rust_subspaces: Vec<Box<dyn AnyStateSpace>> = Vec::with_capacity(subspaces.len());

        Python::with_gil(|py| {
            for obj in subspaces {
                let space_any = obj.bind(py);

                if let Ok(rv_space) = space_any.extract::<PyRef<PyRealVectorStateSpace>>() {
                    rust_subspaces.push(Box::new((*rv_space.0).lock().unwrap().clone()));
                } else if let Ok(so2_space) = space_any.extract::<PyRef<PySO2StateSpace>>() {
                    rust_subspaces.push(Box::new((*so2_space.0).lock().unwrap().clone()));
                } else if let Ok(so3_space) = space_any.extract::<PyRef<PySO3StateSpace>>() {
                    rust_subspaces.push(Box::new((*so3_space.0).lock().unwrap().clone()));
                } else {
                    return Err(PyValueError::new_err(format!(
                        "Object of type '{}' is not a valid state space component.",
                        space_any.get_type().name()?
                    )));
                }
            }
            Ok(())
        })?;
        let space = OxmplCompoundStateSpace::new(rust_subspaces, weights);
        Ok(Self(Rc::new(RefCell::new(space))))
    }

    fn distance(&self, state1: &PyCompoundState, state2: &PyCompoundState) -> f64 {
        self.0.borrow().distance(&state1.0, &state2.0)
    }
}
