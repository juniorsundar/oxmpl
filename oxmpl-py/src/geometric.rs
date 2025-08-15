use pyo3::prelude::*;
use pyo3::types::PyDict;
use std::cell::RefCell;
use std::rc::Rc;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::base::{PyGoal, PyPath, PyProblemDefinition, PyStateValidityChecker};
use oxmpl::base::{planner::Planner, space::RealVectorStateSpace, state::RealVectorState};
use oxmpl::geometric::{RRTConnect, RRTStar, PRM, RRT};

/// A concrete type alias for the RRT planner configured for `RealVectorState`.
type RrtForRealVector = RRT<RealVectorState, RealVectorStateSpace, PyGoal>;
type RrtConnectForRealVector = RRTConnect<RealVectorState, RealVectorStateSpace, PyGoal>;
type RrtStarForRealVector = RRTStar<RealVectorState, RealVectorStateSpace, PyGoal>;
type PrmForRealVector = PRM<RealVectorState, RealVectorStateSpace, PyGoal>;

// Generate the Python wrapper for the RRT planner using the macro.
define_planner!(
    PyRrtRv,             // The Rust wrapper struct name
    "RRT",               // The Python class name
    RrtForRealVector,    // The concrete Rust planner type
    PyProblemDefinition, // The PyO3 ProblemDefinition wrapper
    PyPath,               // The PyO3 Path wrapper
    (max_distance: f64, goal_bias: f64) // Constructor arguments
);

define_planner!(
    PyRrtConnectRv,
    "RRTConnect",
    RrtConnectForRealVector,
    PyProblemDefinition,
    PyPath,
    (max_distance: f64, goal_bias: f64)
);

define_planner!(
    PyRrtStarRv,
    "RRTStar",
    RrtStarForRealVector,
    PyProblemDefinition,
    PyPath,
    (max_distance: f64, goal_bias: f64, search_radius: f64)
);

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
        let problem_def_rust = problem_def_py.0.clone();
        let checker = Arc::new(PyStateValidityChecker {
            callback: validity_callback,
        });
        self.planner.borrow_mut().setup(problem_def_rust, checker);
        Ok(())
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
