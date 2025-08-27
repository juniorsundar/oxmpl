// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::prelude::*;

mod goal;
mod path;
mod problem_definition;
mod py_state_convert;
mod real_vector_state;
mod real_vector_state_space;
mod so2_state;
mod so2_state_space;
mod state_validity_checker;

pub use goal::PyGoal;
pub use path::PyPath;
pub use problem_definition::ProblemDefinitionVariant;
pub use problem_definition::PyProblemDefinition;
pub use real_vector_state::PyRealVectorState;
pub use real_vector_state_space::PyRealVectorStateSpace;
pub use so2_state::PySO2State;
pub use so2_state_space::PySO2StateSpace;
pub use state_validity_checker::PyStateValidityChecker;

pub fn create_module(_py: Python<'_>) -> PyResult<Bound<'_, PyModule>> {
    let base_module = PyModule::new(_py, "base")?;
    base_module.add_class::<PyRealVectorState>()?;
    base_module.add_class::<PyRealVectorStateSpace>()?;
    base_module.add_class::<PySO2State>()?;
    base_module.add_class::<PySO2StateSpace>()?;
    base_module.add_class::<PyPath>()?;
    base_module.add_class::<PyProblemDefinition>()?;
    Ok(base_module)
}
