// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::prelude::*;

mod compound_state;
mod compound_state_space;
mod goal;
mod path;
mod problem_definition;
mod py_state_convert;
mod real_vector_state;
mod real_vector_state_space;
mod se2_state;
mod se2_state_space;
mod se3_state;
mod se3_state_space;
mod so2_state;
mod so2_state_space;
mod so3_state;
mod so3_state_space;
mod state_validity_checker;

pub use compound_state::PyCompoundState;
pub use compound_state_space::PyCompoundStateSpace;
pub use goal::PyGoal;
pub use path::PyPath;
pub use problem_definition::ProblemDefinitionVariant;
pub use problem_definition::PyProblemDefinition;
pub use real_vector_state::PyRealVectorState;
pub use real_vector_state_space::PyRealVectorStateSpace;
pub use se2_state::PySE2State;
pub use se2_state_space::PySE2StateSpace;
pub use se3_state::PySE3State;
pub use se3_state_space::PySE3StateSpace;
pub use so2_state::PySO2State;
pub use so2_state_space::PySO2StateSpace;
pub use so3_state::PySO3State;
pub use so3_state_space::PySO3StateSpace;
pub use state_validity_checker::PyStateValidityChecker;

pub fn create_module(_py: Python<'_>) -> PyResult<Bound<'_, PyModule>> {
    let base_module = PyModule::new(_py, "base")?;
    base_module.add_class::<PySE2State>()?;
    base_module.add_class::<PySE2StateSpace>()?;
    base_module.add_class::<PySE3State>()?;
    base_module.add_class::<PySE3StateSpace>()?;
    base_module.add_class::<PyRealVectorState>()?;
    base_module.add_class::<PyRealVectorStateSpace>()?;
    base_module.add_class::<PyCompoundState>()?;
    base_module.add_class::<PyCompoundStateSpace>()?;
    base_module.add_class::<PySO2State>()?;
    base_module.add_class::<PySO2StateSpace>()?;
    base_module.add_class::<PySO3State>()?;
    base_module.add_class::<PySO3StateSpace>()?;
    base_module.add_class::<PyPath>()?;
    base_module.add_class::<PyProblemDefinition>()?;
    Ok(base_module)
}
