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
        space::{
            CompoundStateSpace, RealVectorStateSpace, SE2StateSpace, SE3StateSpace, SO2StateSpace,
            SO3StateSpace,
        },
        state::{CompoundState, RealVectorState, SE2State, SE3State, SO2State, SO3State},
    },
    geometric::PRM,
};

type PrmForRealVector = PRM<RealVectorState, RealVectorStateSpace, PyGoal<RealVectorState>>;
type PrmForSO2 = PRM<SO2State, SO2StateSpace, PyGoal<SO2State>>;
type PrmForSO3 = PRM<SO3State, SO3StateSpace, PyGoal<SO3State>>;
type PrmForCompound = PRM<CompoundState, CompoundStateSpace, PyGoal<CompoundState>>;
type PrmForSE2 = PRM<SE2State, SE2StateSpace, PyGoal<SE2State>>;
type PrmForSE3 = PRM<SE3State, SE3StateSpace, PyGoal<SE3State>>;

enum PlannerVariant {
    RealVector(Rc<RefCell<PrmForRealVector>>),
    SO2(Rc<RefCell<PrmForSO2>>),
    SO3(Rc<RefCell<PrmForSO3>>),
    Compound(Rc<RefCell<PrmForCompound>>),
    SE2(Rc<RefCell<PrmForSE2>>),
    SE3(Rc<RefCell<PrmForSE3>>),
}

#[pyclass(name = "PRM", unsendable)]
pub struct PyPrm {
    planner: PlannerVariant,
    pd: ProblemDefinitionVariant,
}

#[pymethods]
impl PyPrm {
    /// Creates a new PRM planner instance.
    ///
    /// The constructor inspects the `problem_definition` to determine which
    /// underlying state space to use (e.g., RealVectorStateSpace, SO2StateSpace).
    #[new]
    fn new(
        timeout: f64,
        connection_radius: f64,
        problem_definition: &PyProblemDefinition,
    ) -> PyResult<Self> {
        let (planner, pd) = match &problem_definition.0 {
            ProblemDefinitionVariant::RealVector(pd) => {
                let planner_instance = PrmForRealVector::new(timeout, connection_radius);
                (
                    PlannerVariant::RealVector(Rc::new(RefCell::new(planner_instance))),
                    ProblemDefinitionVariant::RealVector(pd.clone()),
                )
            }
            ProblemDefinitionVariant::SO2(pd) => {
                let planner_instance = PrmForSO2::new(timeout, connection_radius);
                (
                    PlannerVariant::SO2(Rc::new(RefCell::new(planner_instance))),
                    ProblemDefinitionVariant::SO2(pd.clone()),
                )
            }
            ProblemDefinitionVariant::SO3(pd) => {
                let planner_instance = PrmForSO3::new(timeout, connection_radius);
                (
                    PlannerVariant::SO3(Rc::new(RefCell::new(planner_instance))),
                    ProblemDefinitionVariant::SO3(pd.clone()),
                )
            }
            ProblemDefinitionVariant::Compound(pd) => {
                let planner_instance = PrmForCompound::new(timeout, connection_radius);
                (
                    PlannerVariant::Compound(Rc::new(RefCell::new(planner_instance))),
                    ProblemDefinitionVariant::Compound(pd.clone()),
                )
            }
            ProblemDefinitionVariant::SE2(pd) => {
                let planner_instance = PrmForSE2::new(timeout, connection_radius);
                (
                    PlannerVariant::SE2(Rc::new(RefCell::new(planner_instance))),
                    ProblemDefinitionVariant::SE2(pd.clone()),
                )
            }
            ProblemDefinitionVariant::SE3(pd) => {
                let planner_instance = PrmForSE3::new(timeout, connection_radius);
                (
                    PlannerVariant::SE3(Rc::new(RefCell::new(planner_instance))),
                    ProblemDefinitionVariant::SE3(pd.clone()),
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
            PlannerVariant::SE2(planner_variant) => {
                let checker = Arc::new(PyStateValidityChecker {
                    callback: validity_callback,
                });
                if let ProblemDefinitionVariant::SE2(problem_def) = &self.pd {
                    planner_variant
                        .borrow_mut()
                        .setup(problem_def.clone(), checker);
                }
            }
            PlannerVariant::SE3(planner_variant) => {
                let checker = Arc::new(PyStateValidityChecker {
                    callback: validity_callback,
                });
                if let ProblemDefinitionVariant::SE3(problem_def) = &self.pd {
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
            PlannerVariant::SE2(p) => {
                let result = p.borrow_mut().solve(timeout);
                match result {
                    Ok(path) => Ok(PyPath::from(path)),
                    Err(e) => Err(pyo3::exceptions::PyException::new_err(e.to_string())),
                }
            }
            PlannerVariant::SE3(p) => {
                let result = p.borrow_mut().solve(timeout);
                match result {
                    Ok(path) => Ok(PyPath::from(path)),
                    Err(e) => Err(pyo3::exceptions::PyException::new_err(e.to_string())),
                }
            }
        }
    }

    fn construct_roadmap(&mut self) -> PyResult<()> {
        let result = match &mut self.planner {
            PlannerVariant::RealVector(p) => p.borrow_mut().construct_roadmap(),
            PlannerVariant::SO2(p) => p.borrow_mut().construct_roadmap(),
            PlannerVariant::SO3(p) => p.borrow_mut().construct_roadmap(),
            PlannerVariant::Compound(p) => p.borrow_mut().construct_roadmap(),
            PlannerVariant::SE2(p) => p.borrow_mut().construct_roadmap(),
            PlannerVariant::SE3(p) => p.borrow_mut().construct_roadmap(),
        };
        match result {
            Ok(_) => Ok(()),
            Err(e) => Err(pyo3::exceptions::PyException::new_err(e.to_string())),
        }
    }
}
