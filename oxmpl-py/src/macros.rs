// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

/// Generates a complete PyO3 wrapper for a concrete planner type.
///
/// This macro reduces boilerplate by acting as a template for creating Python
/// classes from your core Rust planners. It generates a `#[pyclass]` struct
/// and a `#[pymethods]` implementation with a standard constructor, a `setup`
/// method, and a `solve` method.
///
/// # Arguments
///
/// - `$wrapper_name:ident`: The name for the new Rust wrapper struct (e.g., `PyRrtRv`).
/// - `$python_name:literal`: The name of the class as it will be exposed in Python (e.g., `"RRT"`).
/// - `$concrete_planner_ty:ty`: The full, concrete Rust type of the planner to be wrapped.
///   This type alias should be defined *before* calling the macro.
/// - `$problem_def_py_ty:ty`: The Python wrapper for the `ProblemDefinition`.
/// - `$path_py_ty:ty`: The Python wrapper for the resulting `Path`.
/// - `( $( $arg_name:ident: $arg_ty:ty ),* )`: Repeating list of constructor arguments:
///   `(arg_name: arg_type, ...)`
/// # Usage
///
/// ```rust,ignore
/// // 1. Define a concrete type alias for your generic planner.
/// type RrtForRealVector = RRT<RealVectorState, RealVectorStateSpace, PyGoal>;
///
/// // 2. Invoke the macro with the required types.
/// define_planner!(
///     PyRrtRv,
///     "RRT",
///     RrtForRealVector,
///     PyProblemDefinition,
///     PyPath,
///     (max_distance: f64, goal_bias: f64)
/// );
/// ```
#[macro_export]
macro_rules! define_planner {
    (
        $wrapper_name:ident,
        $python_name:literal,
        $concrete_planner_ty:ty,
        $problem_def_py_ty:ty,
        $path_py_ty:ty,
        ( $( $arg_name:ident: $arg_ty:ty ),* )
    ) => {
        #[pyclass(name = $python_name, unsendable)]
        struct $wrapper_name {
            planner: Arc<Mutex<$concrete_planner_ty>>,
        }

        #[pymethods]
        impl $wrapper_name {
            /// The Python `__init__` constructor.
            #[new]
            fn new( $( $arg_name: $arg_ty ),* ) -> Self {
                let planner_instance = <$concrete_planner_ty>::new( $( $arg_name ),* );
                Self {
                    planner: Arc::new(Mutex::new(planner_instance)),
                }
            }

            /// Configures the planner for a specific problem.
            ///
            /// This method bridges the Python objects (like a function for validity checking) into
            /// the Rust types required by the core `oxmpl` planner.
            fn setup(
                &mut self,
                problem_def_py: &$problem_def_py_ty,
                validity_callback: PyObject,
            ) -> PyResult<()> {
                let problem_def_rust = problem_def_py.0.clone();
                let checker = Arc::new(PyStateValidityChecker {
                    callback: validity_callback,
                });
                match &problem_def_py.0 {
                    ProblemDefinitionVariant::RealVector(pd_arc) => {
                        let checker = Arc::new(PyStateValidityChecker {
                            callback: validity_callback,
                        });

                        self.planner.borrow_mut().setup(pd_arc.clone(), checker);
                        Ok(())
                    }

                    // If the user passes any other kind of problem, raise an error.
                    _ => Err(PyValueError::new_err(
                            "The PRM planner only supports RealVectorStateSpace.",
                    )),
                }
            }

            /// Attempts to solve the planning problem within a given timeout.
            fn solve(&mut self, timeout_secs: f32) -> PyResult<$path_py_ty> {
                let timeout = Duration::from_secs_f32(timeout_secs);
                let result = self.planner.lock().unwrap().solve(timeout);

                // Converts the Rust `Result<Path, PlanningError>` into a Python
                // result, raising an exception on failure.
                match result {
                    Ok(path) => Ok(<$path_py_ty>::from_rust_path(path)),
                    Err(e) => Err(pyo3::exceptions::PyException::new_err(e.to_string())),
                }
            }
        }
    };
}
