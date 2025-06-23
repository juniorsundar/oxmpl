use pyo3::{exceptions::PyValueError, prelude::*};
use rand::Rng;
use std::sync::Arc;

use oxmpl::base::{
    error::StateSamplingError,
    goal::{Goal, GoalRegion, GoalSampleableRegion},
    planner::Path as OxmplPath,
    problem_definition::ProblemDefinition,
    space::{RealVectorStateSpace as OxmplRealVectorStateSpace, StateSpace as _},
    state::RealVectorState as OxmplRealVectorState,
    validity::StateValidityChecker,
};

/// A state in an N-dimensional Euclidean space (R^n).
///
/// This class represents a single point or configuration, defined by a list of floating-point
/// numbers.
///
/// Args:
///     values (List[float]): A list of numbers representing the state's components.
#[pyclass(name = "RealVectorState", unsendable)]
#[derive(Clone)]
pub struct PyRealVectorState(pub Arc<OxmplRealVectorState>);

#[pymethods]
impl PyRealVectorState {
    #[new]
    fn new(values: Vec<f64>) -> Self {
        // Creates the underlying Rust struct and wraps it for Python.
        let state = OxmplRealVectorState::new(values);
        Self(Arc::new(state))
    }

    /// list[float]: The components of the state vector.
    #[getter]
    fn get_values(&self) -> Vec<f64> {
        self.0.values.clone()
    }

    fn __repr__(&self) -> String {
        format!("<RealVectorState values={:?}>", self.0.values)
    }
}

/// Defines an N-dimensional space for `RealVectorState` instances.
///
/// This class defines the planning space, including its dimensionality and boundaries.
#[pyclass(name = "RealVectorStateSpace", unsendable)]
#[derive(Clone)]
pub struct PyRealVectorStateSpace(pub Arc<OxmplRealVectorStateSpace>);

#[pymethods]
impl PyRealVectorStateSpace {
    /// Creates a new `RealVectorStateSpace`.
    ///
    /// Args:
    ///     dimension (int): The number of dimensions for the space.
    ///     bounds (Optional[List[Tuple[float, float]]]): If provided, defines the
    ///         min and max for each dimension. If `None`, the space is unbounded.
    ///
    /// Raises:
    ///     ValueError: If the provided inputs are invalid.
    #[new]
    #[pyo3(signature = (dimension, bounds=None))]
    fn new(dimension: usize, bounds: Option<Vec<(f64, f64)>>) -> PyResult<Self> {
        match OxmplRealVectorStateSpace::new(dimension, bounds) {
            Ok(space) => Ok(Self(Arc::new(space))),
            Err(e) => Err(PyValueError::new_err(e.to_string())),
        }
    }

    /// Computes the Euclidean (L2) distance between two states.
    fn distance(&self, state1: &PyRealVectorState, state2: &PyRealVectorState) -> f64 {
        // This calls the real Rust implementation.
        self.0.distance(&state1.0, &state2.0)
    }
}

/// A sequence of states representing a solution path found by a planner.
#[pyclass(name = "Path")]
#[derive(Clone)]
pub struct PyPath(pub OxmplPath<OxmplRealVectorState>);

#[pymethods]
impl PyPath {
    /// Creates a new Path object from a list of states.
    #[new]
    fn py_new(states: Vec<PyRealVectorState>) -> Self {
        let rust_states = states.into_iter().map(|s| (*s.0).clone()).collect();
        Self(OxmplPath(rust_states))
    }

    /// list[RealVectorState]: The sequence of states that make up the path.
    #[getter]
    fn get_states(&self) -> Vec<PyRealVectorState> {
        self.0
             .0
            .iter()
            .map(|s| PyRealVectorState(Arc::new(s.clone())))
            .collect()
    }

    /// The number of states in the path.
    fn __len__(&self) -> usize {
        self.0 .0.len()
    }

    fn __repr__(&self) -> String {
        format!("<Path with {} states>", self.0 .0.len())
    }
}

impl PyPath {
    /// An internal helper to create a `PyPath` from a core Rust path. Not exposed to Python.
    pub fn from_rust_path(path: OxmplPath<OxmplRealVectorState>) -> Self {
        Self(path)
    }
}

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
        let pd = ProblemDefinition {
            space: space.0.clone(),
            start_states: vec![(*start_state.0).clone()],
            goal: Arc::new(goal_wrapper),
        };
        Self(Arc::new(pd))
    }
}

/// An internal Rust struct that implements the `StateValidityChecker` trait by calling a
/// user-provided Python function.
pub struct PyStateValidityChecker {
    pub callback: PyObject,
}
impl Clone for PyStateValidityChecker {
    fn clone(&self) -> Self {
        Python::with_gil(|py| Self {
            callback: self.callback.clone_ref(py),
        })
    }
}

impl StateValidityChecker<OxmplRealVectorState> for PyStateValidityChecker {
    fn is_valid(&self, state: &OxmplRealVectorState) -> bool {
        Python::with_gil(|py| {
            let result: PyResult<bool> = (move || {
                let py_state = Py::new(py, PyRealVectorState(Arc::new(state.clone())))?;
                let args = (py_state,);
                let result = self.callback.call1(py, args)?;
                result.extract(py)
            })();
            match result {
                Ok(is_valid) => is_valid,
                Err(e) => {
                    e.print(py);
                    false
                }
            }
        })
    }
}

/// An internal Rust struct that implements the `Goal` trait hierarchy
/// by calling methods on a user-provided Python goal object.
pub struct PyGoal {
    pub instance: PyObject,
}
impl Clone for PyGoal {
    fn clone(&self) -> Self {
        Python::with_gil(|py| Self {
            instance: self.instance.clone_ref(py),
        })
    }
}

impl Goal<OxmplRealVectorState> for PyGoal {
    fn is_satisfied(&self, state: &OxmplRealVectorState) -> bool {
        Python::with_gil(|py| {
            let py_state = PyRealVectorState(Arc::new(state.clone()));
            self.instance
                .call_method1(py, "is_satisfied", (py_state,))
                .and_then(|res| res.extract(py))
                .unwrap_or(false)
        })
    }
}
impl GoalRegion<OxmplRealVectorState> for PyGoal {
    fn distance_goal(&self, state: &OxmplRealVectorState) -> f64 {
        Python::with_gil(|py| {
            let py_state = PyRealVectorState(Arc::new(state.clone()));
            self.instance
                .call_method1(py, "distance_goal", (py_state,))
                .and_then(|res| res.extract(py))
                .unwrap_or(f64::INFINITY)
        })
    }
}
impl GoalSampleableRegion<OxmplRealVectorState> for PyGoal {
    fn sample_goal(&self, _rng: &mut impl Rng) -> Result<OxmplRealVectorState, StateSamplingError> {
        Python::with_gil(|py| {
            self.instance
                .call_method0(py, "sample_goal")
                .and_then(|res| res.extract::<PyRealVectorState>(py))
                .map(|py_state| (*py_state.0).clone())
                .map_err(|e| {
                    e.print(py);
                    StateSamplingError::GoalRegionUnsatisfiable
                })
        })
    }
}

// --- Function to build and register the `base` submodule ---
pub fn create_module(_py: Python<'_>) -> PyResult<Bound<'_, PyModule>> {
    let base_module = PyModule::new(_py, "base")?;
    base_module.add_class::<PyRealVectorState>()?;
    base_module.add_class::<PyRealVectorStateSpace>()?;
    base_module.add_class::<PyPath>()?;
    base_module.add_class::<PyProblemDefinition>()?;
    // parent_module.add_submodule(&base_module)?;
    Ok(base_module)
}
