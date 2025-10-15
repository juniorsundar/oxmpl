// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use pyo3::{exceptions::PyValueError, prelude::*};
use std::{cell::RefCell, rc::Rc};

use oxmpl::base::space::{SE3StateSpace as OxmplSE3StateSpace, StateSpace as _};

use super::se3_state::PySE3State;

/// Defines the state space for 3D rigid body motion (SE(3)).
///
/// This space combines a 3D translational component (`RealVectorStateSpace`) and a 3D rotational
/// component (`SO3StateSpace`).
#[pyclass(name = "SE3StateSpace", unsendable)]
#[derive(Clone)]
pub struct PySE3StateSpace(pub Rc<RefCell<OxmplSE3StateSpace>>);

#[pymethods]
impl PySE3StateSpace {
    /// Creates a new `SE3StateSpace`.
    ///
    /// The distance between two states in this space is a weighted sum of the Euclidean distance
    /// of the translational part and the angular distance of the rotational part.
    ///
    /// Args:
    ///     weight (float): The weighting factor applied to the rotational distance.
    ///     bounds (Optional[List[Tuple[float, float]]]): If provided, defines the
    ///         min and max for the x, y, and z dimensions. If `None`, the translational
    ///         space is unbounded.
    ///
    /// Raises:
    ///     ValueError: If the provided bounds are invalid.
    #[new]
    #[pyo3(signature = (weight, bounds=None))]
    fn new(weight: f64, bounds: Option<Vec<(f64, f64)>>) -> PyResult<Self> {
        match OxmplSE3StateSpace::new(weight, bounds) {
            Ok(space) => Ok(Self(Rc::new(RefCell::new(space)))),
            Err(e) => Err(PyValueError::new_err(e.to_string())),
        }
    }

    /// Computes the weighted distance between two SE(3) states.
    fn distance(&self, state1: &PySE3State, state2: &PySE3State) -> f64 {
        self.0.borrow().distance(&state1.0, &state2.0)
    }
}
