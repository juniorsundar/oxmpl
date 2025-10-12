// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::prelude::*;
use std::{cell::RefCell, rc::Rc, sync::Arc, time::Duration};

use crate::base::{
    ProblemDefinitionVariant, PyGoal, PyPath, PyProblemDefinition, PyStateValidityChecker,
};
use oxmpl::{
    base::{
        planner::Planner,
        space::{CompoundStateSpace, RealVectorStateSpace, SO2StateSpace, SO3StateSpace},
        state::{CompoundState, RealVectorState, SO2State, SO3State},
    },
    geometric::RRTStar,
};

type RrtStarForRealVector = RRTStar<RealVectorState, RealVectorStateSpace, PyGoal<RealVectorState>>;
type RrtStarForSO2 = RRTStar<SO2State, SO2StateSpace, PyGoal<SO2State>>;
type RrtStarForSO3 = RRTStar<SO3State, SO3StateSpace, PyGoal<SO3State>>;
type RrtStarForCompound = RRTStar<CompoundState, CompoundStateSpace, PyGoal<CompoundState>>;

enum PlannerVariant {
    RealVector(Rc<RefCell<RrtStarForRealVector>>),
    SO2(Rc<RefCell<RrtStarForSO2>>),
    SO3(Rc<RefCell<RrtStarForSO3>>),
    Compound(Rc<RefCell<RrtStarForCompound>>),
}

#[pyclass(name = "RRTStar", unsendable)]
pub struct PyRrtStar {
    planner: PlannerVariant,
    pd: ProblemDefinitionVariant,
}

#[pymethods]
impl PyRrtStar {
    /// Creates a new RRTStar planner instance.
    ///
    /// The constructor inspects the `problem_definition` to determine which
    /// underlying state space to use (e.g., RealVectorStateSpace, SO2StateSpace).
    #[new]
    fn new(
        max_distance: f64,
        goal_bias: f64,
        search_radius: f64,
        problem_definition: &PyProblemDefinition,
    ) -> PyResult<Self> {
        let (planner, pd) = match &problem_definition.0 {
            ProblemDefinitionVariant::RealVector(pd) => {
                let planner_instance =
                    RrtStarForRealVector::new(max_distance, goal_bias, search_radius);
                (
                    PlannerVariant::RealVector(Rc::new(RefCell::new(planner_instance))),
                    ProblemDefinitionVariant::RealVector(pd.clone()),
                )
            }
            ProblemDefinitionVariant::SO2(pd) => {
                let planner_instance = RrtStarForSO2::new(max_distance, goal_bias, search_radius);
                (
                    PlannerVariant::SO2(Rc::new(RefCell::new(planner_instance))),
                    ProblemDefinitionVariant::SO2(pd.clone()),
                )
            }
            ProblemDefinitionVariant::SO3(pd) => {
                let planner_instance = RrtStarForSO3::new(max_distance, goal_bias, search_radius);
                (
                    PlannerVariant::SO3(Rc::new(RefCell::new(planner_instance))),
                    ProblemDefinitionVariant::SO3(pd.clone()),
                )
            }
            ProblemDefinitionVariant::Compound(pd) => {
                let planner_instance =
                    RrtStarForCompound::new(max_distance, goal_bias, search_radius);
                (
                    PlannerVariant::Compound(Rc::new(RefCell::new(planner_instance))),
                    ProblemDefinitionVariant::Compound(pd.clone()),
                )
            }
        };
        Ok(Self { planner, pd })
    }

    fn setup(&mut self, validity_callback: PyObject) -> PyResult<()> {
        match &mut self.planner {
            PlannerVariant::RealVector(planner_variant) => {
                let checker = Arc::new(PyStateValidityChecker {
                    callback: validity_callback,
                });
                if let ProblemDefinitionVariant::RealVector(problem_def) = &self.pd {
                    planner_variant
                        .borrow_mut()
                        .setup(problem_def.clone(), checker);
                }
            }
            PlannerVariant::SO2(planner_variant) => {
                let checker = Arc::new(PyStateValidityChecker {
                    callback: validity_callback,
                });
                if let ProblemDefinitionVariant::SO2(problem_def) = &self.pd {
                    planner_variant
                        .borrow_mut()
                        .setup(problem_def.clone(), checker);
                }
            }
            PlannerVariant::SO3(planner_variant) => {
                let checker = Arc::new(PyStateValidityChecker {
                    callback: validity_callback,
                });
                if let ProblemDefinitionVariant::SO3(problem_def) = &self.pd {
                    planner_variant
                        .borrow_mut()
                        .setup(problem_def.clone(), checker);
                }
            }
            PlannerVariant::Compound(planner_variant) => {
                let checker = Arc::new(PyStateValidityChecker {
                    callback: validity_callback,
                });
                if let ProblemDefinitionVariant::Compound(problem_def) = &self.pd {
                    planner_variant
                        .borrow_mut()
                        .setup(problem_def.clone(), checker);
                }
            }
        }
        Ok(())
    }

    fn solve(&mut self, timeout_secs: f32) -> PyResult<PyPath> {
        let timeout = Duration::from_secs_f32(timeout_secs);
        match &mut self.planner {
            PlannerVariant::RealVector(p) => {
                let result = p.borrow_mut().solve(timeout);
                match result {
                    Ok(path) => Ok(PyPath::from(path)),
                    Err(e) => Err(pyo3::exceptions::PyException::new_err(e.to_string())),
                }
            }
            PlannerVariant::SO2(p) => {
                let result = p.borrow_mut().solve(timeout);
                match result {
                    Ok(path) => Ok(PyPath::from(path)),
                    Err(e) => Err(pyo3::exceptions::PyException::new_err(e.to_string())),
                }
            }
            PlannerVariant::SO3(p) => {
                let result = p.borrow_mut().solve(timeout);
                match result {
                    Ok(path) => Ok(PyPath::from(path)),
                    Err(e) => Err(pyo3::exceptions::PyException::new_err(e.to_string())),
                }
            }
            PlannerVariant::Compound(p) => {
                let result = p.borrow_mut().solve(timeout);
                match result {
                    Ok(path) => Ok(PyPath::from(path)),
                    Err(e) => Err(pyo3::exceptions::PyException::new_err(e.to_string())),
                }
            }
        }
    }
}
