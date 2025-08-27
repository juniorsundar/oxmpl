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
        space::{RealVectorStateSpace, SO2StateSpace},
        state::{RealVectorState, SO2State},
    },
    geometric::RRTStar,
};

type RrtStarForRealVector = RRTStar<RealVectorState, RealVectorStateSpace, PyGoal<RealVectorState>>;
type RrtStarForSO2 = RRTStar<SO2State, SO2StateSpace, PyGoal<SO2State>>;

enum PlannerVariant {
    RealVector(Rc<RefCell<RrtStarForRealVector>>),
    SO2(Rc<RefCell<RrtStarForSO2>>),
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
        };
        Ok(Self { planner, pd })
    }

    fn setup(&mut self, validity_callback: PyObject) -> PyResult<()> {
        match &mut self.planner {
            PlannerVariant::RealVector(prm_rv) => {
                let checker = Arc::new(PyStateValidityChecker {
                    callback: validity_callback,
                });
                if let ProblemDefinitionVariant::RealVector(problem_def) = &self.pd {
                    prm_rv.borrow_mut().setup(problem_def.clone(), checker);
                }
            }
            PlannerVariant::SO2(prm_so2) => {
                let checker = Arc::new(PyStateValidityChecker {
                    callback: validity_callback,
                });
                if let ProblemDefinitionVariant::SO2(problem_def) = &self.pd {
                    prm_so2.borrow_mut().setup(problem_def.clone(), checker);
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
        }
    }
}
