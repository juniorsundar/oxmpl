use pyo3::prelude::*;
use pyo3::types::PyDict;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::base::{PyGoal, PyPath, PyProblemDefinition, PyStateValidityChecker};
use oxmpl::base::{planner::Planner, space::RealVectorStateSpace, state::RealVectorState};
use oxmpl::geometric::planners::rrt::RRT;

/// A concrete type alias for the RRT planner configured for `RealVectorState`.
type RrtForRealVector = RRT<RealVectorState, RealVectorStateSpace, PyGoal>;

// Generate the Python wrapper for the RRT planner using the macro.
define_planner!(
    PyRrtRv,             // The Rust wrapper struct name
    "RRT",               // The Python class name
    RrtForRealVector,    // The concrete Rust planner type
    PyProblemDefinition, // The PyO3 ProblemDefinition wrapper
    PyPath               // The PyO3 Path wrapper
);

pub fn create_module(py: Python<'_>) -> PyResult<Bound<'_, PyModule>> {
    let geometric_module = PyModule::new(py, "geometric")?;
    let planners_module = PyModule::new(py, "planners")?;

    let sys_modules = py
        .import("sys")?
        .getattr("modules")?
        .downcast_into::<PyDict>()?;

    planners_module.add_class::<PyRrtRv>()?;
    geometric_module.add_submodule(&planners_module)?;

    sys_modules.set_item("oxmpl_py.geometric.planners", planners_module)?;

    Ok(geometric_module)
}
