// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::{exceptions::PyValueError, prelude::*};
use std::{cell::RefCell, rc::Rc};

use oxmpl::base::space::{SE2StateSpace as OxmplSE2StateSpace, StateSpace as _};

use super::se2_state::PySE2State;

/// Defines the state space for 2D rigid body motion (SE(2)).
///
/// This space combines a 2D translational component (`RealVectorStateSpace`) and a 1D rotational
/// component (`SO2StateSpace`).
#[pyclass(name = "SE2StateSpace", unsendable)]
#[derive(Clone)]
pub struct PySE2StateSpace(pub Rc<RefCell<OxmplSE2StateSpace>>);

#[pymethods]
impl PySE2StateSpace {
    /// Creates a new `SE2StateSpace`.
    ///
    /// The distance between two states in this space is a weighted sum of the Euclidean distance
    /// of the translational part and the angular distance of the rotational part.
    ///
    /// Args:
    ///     weight (float): The weighting factor applied to the rotational distance.
    ///     bounds (Optional[List[Tuple[float, float]]]): If provided, defines the
    ///         min and max for the x and y dimensions. If `None`, the translational
    ///         space is unbounded. The rotational space is always `[-PI, PI)`.
    ///
    /// Raises:
    ///     ValueError: If the provided bounds are invalid.
    #[new]
    #[pyo3(signature = (weight, bounds=None))]
    fn new(weight: f64, bounds: Option<Vec<(f64, f64)>>) -> PyResult<Self> {
        match OxmplSE2StateSpace::new(weight, bounds) {
            Ok(space) => Ok(Self(Rc::new(RefCell::new(space)))),
            Err(e) => Err(PyValueError::new_err(e.to_string())),
        }
    }

    /// Computes the weighted distance between two SE(2) states.
    fn distance(&self, state1: &PySE2State, state2: &PySE2State) -> f64 {
        self.0.borrow().distance(&state1.0, &state2.0)
    }
}
