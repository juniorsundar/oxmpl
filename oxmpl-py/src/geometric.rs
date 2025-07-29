use pyo3::prelude::*;
use pyo3::types::PyDict;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::base::{PyGoal, PyPath, PyProblemDefinition, PyStateValidityChecker};
use oxmpl::base::{planner::Planner, space::RealVectorStateSpace, state::RealVectorState};
use oxmpl::geometric::{RRT, RRTConnect};

/// A concrete type alias for the RRT planner configured for `RealVectorState`.
type RrtForRealVector = RRT<RealVectorState, RealVectorStateSpace, PyGoal>;
type RrtConnectForRealVector = RRTConnect<RealVectorState, RealVectorStateSpace, PyGoal>;

// Generate the Python wrapper for the RRT planner using the macro.
define_planner!(
    PyRrtRv,             // The Rust wrapper struct name
    "RRT",               // The Python class name
    RrtForRealVector,    // The concrete Rust planner type
    PyProblemDefinition, // The PyO3 ProblemDefinition wrapper
    PyPath               // The PyO3 Path wrapper
);

define_planner!(
    PyRrtConnectRv,
    "RRTConnect",
    RrtConnectForRealVector,
    PyProblemDefinition,
    PyPath
);

pub fn create_module(py: Python<'_>) -> PyResult<Bound<'_, PyModule>> {
    let geometric_module = PyModule::new(py, "geometric")?;

    let sys_modules = py
        .import("sys")?
        .getattr("modules")?
        .downcast_into::<PyDict>()?;
    geometric_module.add_class::<PyRrtConnectRv>()?;
    geometric_module.add_class::<PyRrtRv>()?;

    sys_modules.set_item("oxmpl_py.geometric", &geometric_module)?;

    Ok(geometric_module)
}
