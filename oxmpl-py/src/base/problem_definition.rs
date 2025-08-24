// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::prelude::*;
use std::sync::Arc;

use oxmpl::base::{
    problem_definition::ProblemDefinition, space::RealVectorStateSpace as OxmplRealVectorStateSpace,
    state::RealVectorState as OxmplRealVectorState,
};

use super::{
    goal::PyGoal, real_vector_state::PyRealVectorState,
    real_vector_state_space::PyRealVectorStateSpace,
};

/// Encapsulates all the components of a motion planning problem.
///
/// This class bundles the state space, start state, and goal definition, which can then be passed
/// to a planner's `setup` method.
#[pyclass(name = "ProblemDefinition", unsendable)]
#[derive(Clone)]
pub struct PyProblemDefinition(
    pub Arc<ProblemDefinition<OxmplRealVectorState, OxmplRealVectorStateSpace, PyGoal>>,
);

#[pymethods]
impl PyProblemDefinition {
    #[new]
    /// Creates a new `ProblemDefinition`.
    ///
    /// Args:
    ///     space (RealVectorStateSpace): The space in which to plan.
    ///     start_state (RealVectorState): The starting configuration.
    ///     goal (object): A Python object that implements the required goal methods
    ///         (`is_satisfied`, `distance_goal`, `sample_goal`).
    fn new(space: PyRealVectorStateSpace, start_state: PyRealVectorState, goal: PyObject) -> Self {
        let goal_wrapper = PyGoal { instance: goal };
        let cloned_inner_space = space.0.lock().unwrap().clone();

        let pd = ProblemDefinition {
            space: Arc::new(cloned_inner_space),
            start_states: vec![(*start_state.0).clone()],
            goal: Arc::new(goal_wrapper),
        };
        Self(Arc::new(pd))
    }
}

