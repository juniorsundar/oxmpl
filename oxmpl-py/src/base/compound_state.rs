// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::{prelude::*, types::PyList};
use std::{any::Any, rc::Rc, sync::Arc};

use oxmpl::base::state::{
    CompoundState as OxmplCompoundState, RealVectorState as OxmplRealVectorState,
    SO2State as OxmplSO2State, SO3State as OxmplSO3State, State,
};

use crate::base::{PyRealVectorState, PySO2State, PySO3State};

#[pyclass(name = "CompoundState", unsendable)]
#[derive(Clone)]
pub struct PyCompoundState(pub Rc<OxmplCompoundState>);

#[pymethods]
impl PyCompoundState {
    #[new]
    fn new(components: Vec<PyObject>) -> PyResult<Self> {
        let mut rust_components: Vec<Box<dyn State>> = Vec::with_capacity(components.len());

        Python::with_gil(|py| {
            for comp_object in components {
                let comp_any = comp_object.bind(py);
                if let Ok(rv_state) = comp_any.extract::<PyRef<PyRealVectorState>>() {
                    rust_components.push(Box::new((*rv_state.0).clone()));
                } else if let Ok(so2_state) = comp_any.extract::<PyRef<PySO2State>>() {
                    rust_components.push(Box::new((*so2_state.0).clone()));
                } else if let Ok(so3_state) = comp_any.extract::<PyRef<PySO3State>>() {
                    rust_components.push(Box::new((*so3_state.0).clone()));
                } else {
                    return Err(pyo3::exceptions::PyValueError::new_err(format!(
                        "Object of type '{}' is not a valid state component.",
                        comp_any.get_type().name()?
                    )));
                }
            }
            Ok(())
        })?;
        let compound_state = OxmplCompoundState {
            components: rust_components,
        };
        Ok(Self(Rc::new(compound_state)))
    }

    #[getter]
    fn get_components(&self, py: Python<'_>) -> PyResult<PyObject> {
        let list = PyList::empty(py);

        for component in &self.0.components {
            let component_any = &**component as &dyn Any;

            if let Some(rv_state) = component_any.downcast_ref::<OxmplRealVectorState>() {
                list.append(Py::new(py, PyRealVectorState(Arc::new(rv_state.clone())))?)?;
            } else if let Some(so2_state) = component_any.downcast_ref::<OxmplSO2State>() {
                list.append(Py::new(py, PySO2State(Arc::new(so2_state.clone())))?)?;
            } else if let Some(so3_state) = component_any.downcast_ref::<OxmplSO3State>() {
                list.append(Py::new(py, PySO3State(Arc::new(so3_state.clone())))?)?;
            } else {
                return Err(pyo3::exceptions::PyTypeError::new_err(
                    "Encountered an unknown state type when getting components.",
                ));
            }
        }

        Ok(list.into())
    }

    fn __len__(&self) -> usize {
        self.0.components.len()
    }

    fn __repr__(&self) -> String {
        format!(
            "<CompoundState with {} components>",
            self.0.components.len()
        )
    }
}
