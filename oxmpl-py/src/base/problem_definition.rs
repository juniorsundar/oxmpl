// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::prelude::*;
use pyo3::types::PyType;
use std::{marker::PhantomData, sync::Arc};

use oxmpl::base::{
    problem_definition::ProblemDefinition,
    space::{
        RealVectorStateSpace as OxmplRealVectorStateSpace, SO2StateSpace as OxmplSO2StateSpace,
        SO3StateSpace as OxmplSO3StateSpace,
    },
    state::{
        RealVectorState as OxmplRealVectorState, SO2State as OxmplSO2State,
        SO3State as OxmplSO3State,
    },
};

use super::{
    goal::PyGoal, real_vector_state::PyRealVectorState,
    real_vector_state_space::PyRealVectorStateSpace, so2_state::PySO2State,
    so2_state_space::PySO2StateSpace, so3_state::PySO3State, so3_state_space::PySO3StateSpace,
};

#[derive(Clone)]
pub enum ProblemDefinitionVariant {
    RealVector(
        Arc<
            ProblemDefinition<
                OxmplRealVectorState,
                OxmplRealVectorStateSpace,
                PyGoal<OxmplRealVectorState>,
            >,
        >,
    ),
    SO2(Arc<ProblemDefinition<OxmplSO2State, OxmplSO2StateSpace, PyGoal<OxmplSO2State>>>),
    SO3(Arc<ProblemDefinition<OxmplSO3State, OxmplSO3StateSpace, PyGoal<OxmplSO3State>>>),
}

/// Encapsulates all the components of a motion planning problem.
#[pyclass(name = "ProblemDefinition", unsendable)]
#[derive(Clone)]
pub struct PyProblemDefinition(pub ProblemDefinitionVariant);

#[pymethods]
impl PyProblemDefinition {
    /// Creates a ProblemDefinition for a RealVectorStateSpace.
    #[classmethod]
    #[pyo3(signature = (space, start_state, goal))]
    fn from_real_vector(
        _cls: &Bound<'_, PyType>,
        space: &PyRealVectorStateSpace,
        start_state: &PyRealVectorState,
        goal: PyObject,
    ) -> Self {
        // Instantiate the correct generic version of PyGoal
        let goal_wrapper = PyGoal::<OxmplRealVectorState> {
            instance: goal,
            _phantom: PhantomData,
        };

        // Create a snapshot of the space's configuration
        let cloned_inner_space = space.0.lock().unwrap().clone();

        let pd = ProblemDefinition {
            space: Arc::new(cloned_inner_space),
            start_states: vec![(*start_state.0).clone()],
            goal: Arc::new(goal_wrapper),
        };

        // Wrap the result in the correct enum variant
        Self(ProblemDefinitionVariant::RealVector(Arc::new(pd)))
    }

    /// Creates a ProblemDefinition for an SO2StateSpace.
    #[classmethod]
    #[pyo3(signature = (space, start_state, goal))]
    fn from_so2(
        _cls: &Bound<'_, PyType>,
        space: &PySO2StateSpace,
        start_state: &PySO2State,
        goal: PyObject,
    ) -> Self {
        // Instantiate the correct generic version of PyGoal
        let goal_wrapper = PyGoal::<OxmplSO2State> {
            instance: goal,
            _phantom: PhantomData,
        };

        // Create a snapshot of the space's configuration
        let cloned_inner_space = space.0.lock().unwrap().clone();

        let pd = ProblemDefinition {
            space: Arc::new(cloned_inner_space),
            start_states: vec![(*start_state.0).clone()],
            goal: Arc::new(goal_wrapper),
        };

        // Wrap the result in the correct enum variant
        Self(ProblemDefinitionVariant::SO2(Arc::new(pd)))
    }

    /// Creates a ProblemDefinition for an SO3StateSpace.
    #[classmethod]
    #[pyo3(signature = (space, start_state, goal))]
    fn from_so3(
        _cls: &Bound<'_, PyType>,
        space: &PySO3StateSpace,
        start_state: &PySO3State,
        goal: PyObject,
    ) -> Self {
        // Instantiate the correct generic version of PyGoal
        let goal_wrapper = PyGoal::<OxmplSO3State> {
            instance: goal,
            _phantom: PhantomData,
        };

        // Create a snapshot of the space's configuration
        let cloned_inner_space = space.0.lock().unwrap().clone();

        let pd = ProblemDefinition {
            space: Arc::new(cloned_inner_space),
            start_states: vec![(*start_state.0).clone()],
            goal: Arc::new(goal_wrapper),
        };

        // Wrap the result in the correct enum variant
        Self(ProblemDefinitionVariant::SO3(Arc::new(pd)))
    }
}
