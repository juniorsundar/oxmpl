// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use std::cell::RefCell;
use std::rc::Rc;
use std::sync::Arc;
use std::time::Duration;

use crate::base::{
    ProblemDefinitionVariant, PyGoal, PyPath, PyProblemDefinition, PyStateValidityChecker,
};
use oxmpl::base::{planner::Planner, space::RealVectorStateSpace, state::RealVectorState};
use oxmpl::geometric::{RRTConnect, RRTStar, PRM, RRT};

type RrtForRealVector = RRT<RealVectorState, RealVectorStateSpace, PyGoal<RealVectorState>>;
type RrtConnectForRealVector =
    RRTConnect<RealVectorState, RealVectorStateSpace, PyGoal<RealVectorState>>;
type RrtStarForRealVector = RRTStar<RealVectorState, RealVectorStateSpace, PyGoal<RealVectorState>>;
type PrmForRealVector = PRM<RealVectorState, RealVectorStateSpace, PyGoal<RealVectorState>>;

#[pyclass(name = "RRT", unsendable)]
struct PyRrtRv {
    planner: Rc<RefCell<RrtForRealVector>>,
}
#[pymethods]
impl PyRrtRv {
    #[doc = r" The Python `__init__` constructor."]
    #[new]
    fn new(max_distance: f64, goal_bias: f64) -> Self {
        let planner_instance = <RrtForRealVector>::new(max_distance, goal_bias);
        Self {
            planner: Rc::new(RefCell::new(planner_instance)),
        }
    }
    #[doc = r" Configures the planner for a specific problem."]
    #[doc = r""]
    #[doc = r" This method bridges the Python objects (like a function for validity checking) into"]
    #[doc = r" the Rust types required by the core `oxmpl` planner."]
    fn setup(
        &mut self,
        problem_def_py: &PyProblemDefinition,
        validity_callback: PyObject,
    ) -> PyResult<()> {
        match &problem_def_py.0 {
            ProblemDefinitionVariant::RealVector(pd_arc) => {
                let checker = Arc::new(PyStateValidityChecker {
                    callback: validity_callback,
                });
                self.planner.borrow_mut().setup(pd_arc.clone(), checker);
                Ok(())
            }
            _ => Err(PyValueError::new_err(
                "The RRT planner only supports RealVectorStateSpace.",
            )),
        }
    }
    #[doc = r" Attempts to solve the planning problem within a given timeout."]
    fn solve(&mut self, timeout_secs: f32) -> PyResult<PyPath> {
        let timeout = Duration::from_secs_f32(timeout_secs);
        let result = self.planner.borrow_mut().solve(timeout);
        match result {
            Ok(path) => Ok(<PyPath>::from_rust_path(path)),
            Err(e) => Err(pyo3::exceptions::PyException::new_err(e.to_string())),
        }
    }
}

#[pyclass(name = "RRTConnect", unsendable)]
struct PyRrtConnectRv {
    planner: Rc<RefCell<RrtConnectForRealVector>>,
}
#[pymethods]
impl PyRrtConnectRv {
    #[doc = r" The Python `__init__` constructor."]
    #[new]
    fn new(max_distance: f64, goal_bias: f64) -> Self {
        let planner_instance = <RrtConnectForRealVector>::new(max_distance, goal_bias);
        Self {
            planner: Rc::new(RefCell::new(planner_instance)),
        }
    }
    #[doc = r" Configures the planner for a specific problem."]
    #[doc = r""]
    #[doc = r" This method bridges the Python objects (like a function for validity checking) into"]
    #[doc = r" the Rust types required by the core `oxmpl` planner."]
    fn setup(
        &mut self,
        problem_def_py: &PyProblemDefinition,
        validity_callback: PyObject,
    ) -> PyResult<()> {
        match &problem_def_py.0 {
            ProblemDefinitionVariant::RealVector(pd_arc) => {
                let checker = Arc::new(PyStateValidityChecker {
                    callback: validity_callback,
                });
                self.planner.borrow_mut().setup(pd_arc.clone(), checker);
                Ok(())
            }
            _ => Err(PyValueError::new_err(
                "The RRT-Connect planner only supports RealVectorStateSpace.",
            )),
        }
    }
    #[doc = r" Attempts to solve the planning problem within a given timeout."]
    fn solve(&mut self, timeout_secs: f32) -> PyResult<PyPath> {
        let timeout = Duration::from_secs_f32(timeout_secs);
        let result = self.planner.borrow_mut().solve(timeout);
        match result {
            Ok(path) => Ok(<PyPath>::from_rust_path(path)),
            Err(e) => Err(pyo3::exceptions::PyException::new_err(e.to_string())),
        }
    }
}

#[pyclass(name = "RRTStar", unsendable)]
struct PyRrtStarRv {
    planner: Rc<RefCell<RrtStarForRealVector>>,
}
#[pymethods]
impl PyRrtStarRv {
    #[doc = r" The Python `__init__` constructor."]
    #[new]
    fn new(max_distance: f64, goal_bias: f64, search_radius: f64) -> Self {
        let planner_instance = <RrtStarForRealVector>::new(max_distance, goal_bias, search_radius);
        Self {
            planner: Rc::new(RefCell::new(planner_instance)),
        }
    }
    #[doc = r" Configures the planner for a specific problem."]
    #[doc = r""]
    #[doc = r" This method bridges the Python objects (like a function for validity checking) into"]
    #[doc = r" the Rust types required by the core `oxmpl` planner."]
    fn setup(
        &mut self,
        problem_def_py: &PyProblemDefinition,
        validity_callback: PyObject,
    ) -> PyResult<()> {
        match &problem_def_py.0 {
            ProblemDefinitionVariant::RealVector(pd_arc) => {
                let checker = Arc::new(PyStateValidityChecker {
                    callback: validity_callback,
                });
                self.planner.borrow_mut().setup(pd_arc.clone(), checker);
                Ok(())
            }
            _ => Err(PyValueError::new_err(
                "The RRT* planner only supports RealVectorStateSpace.",
            )),
        }
    }
    #[doc = r" Attempts to solve the planning problem within a given timeout."]
    fn solve(&mut self, timeout_secs: f32) -> PyResult<PyPath> {
        let timeout = Duration::from_secs_f32(timeout_secs);
        let result = self.planner.borrow_mut().solve(timeout);
        match result {
            Ok(path) => Ok(<PyPath>::from_rust_path(path)),
            Err(e) => Err(pyo3::exceptions::PyException::new_err(e.to_string())),
        }
    }
}

#[pyclass(name = "PRM", unsendable)]
struct PyPrmRv {
    planner: Rc<RefCell<PrmForRealVector>>,
}
#[pymethods]
impl PyPrmRv {
    #[doc = r" The Python `__init__` constructor."]
    #[new]
    fn new(timeout: f64, connection_radius: f64) -> Self {
        let planner_instance = <PrmForRealVector>::new(timeout, connection_radius);
        Self {
            planner: Rc::new(RefCell::new(planner_instance)),
        }
    }
    #[doc = r" Configures the planner for a specific problem."]
    #[doc = r""]
    #[doc = r" This method bridges the Python objects (like a function for validity checking) into"]
    #[doc = r" the Rust types required by the core `oxmpl` planner."]
    fn setup(
        &mut self,
        problem_def_py: &PyProblemDefinition,
        validity_callback: PyObject,
    ) -> PyResult<()> {
        match &problem_def_py.0 {
            ProblemDefinitionVariant::RealVector(pd_arc) => {
                let checker = Arc::new(PyStateValidityChecker {
                    callback: validity_callback,
                });

                self.planner.borrow_mut().setup(pd_arc.clone(), checker);
                Ok(())
            }

            // If the user passes any other kind of problem, raise an error.
            _ => Err(PyValueError::new_err(
                "The PRM planner only supports RealVectorStateSpace.",
            )),
        }
    }
    #[doc = r" Attempts to solve the planning problem within a given timeout."]
    fn solve(&mut self, timeout_secs: f32) -> PyResult<PyPath> {
        let timeout = Duration::from_secs_f32(timeout_secs);
        let result = self.planner.borrow_mut().solve(timeout);
        match result {
            Ok(path) => Ok(<PyPath>::from_rust_path(path)),
            Err(e) => Err(pyo3::exceptions::PyException::new_err(e.to_string())),
        }
    }

    #[doc = r" Construct roadmap."]
    fn construct_roadmap(&mut self) -> PyResult<()> {
        let result = self.planner.borrow_mut().construct_roadmap();
        match result {
            Ok(_) => Ok(()),
            Err(e) => Err(pyo3::exceptions::PyException::new_err(e.to_string())),
        }
    }
}

pub fn create_module(py: Python<'_>) -> PyResult<Bound<'_, PyModule>> {
    let geometric_module = PyModule::new(py, "geometric")?;

    let sys_modules = py
        .import("sys")?
        .getattr("modules")?
        .downcast_into::<PyDict>()?;
    geometric_module.add_class::<PyRrtRv>()?;
    geometric_module.add_class::<PyRrtConnectRv>()?;
    geometric_module.add_class::<PyRrtStarRv>()?;
    geometric_module.add_class::<PyPrmRv>()?;

    sys_modules.set_item("oxmpl_py.geometric", &geometric_module)?;

    Ok(geometric_module)
}
