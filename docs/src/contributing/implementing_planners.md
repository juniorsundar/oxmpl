# Implementing New Planners
This guide outlines the steps to add a new planning algorithm to OxMPL and expose it to the Python and JavaScript bindings.

## Rust Implementation (`oxmpl`)
1.  **Create the Planner Struct**: Define your planner struct in `oxmpl/src/geometric/planners/<your_planner>.rs`.
2.  **Implement `Planner` Trait**: Implement the `oxmpl::base::planner::Planner` trait for your struct.
    *   `setup`: Initialize the planner with the problem definition.
    *   `solve`: The core logic to find a path.
    *   `get_planner_data`: Return internal data (like the graph) for visualization/debugging.
3.  **Export**: Add your planner to `oxmpl/src/geometric/planners/mod.rs` and re-export it in `oxmpl/src/geometric/mod.rs`.

```rust,ignore
// oxmpl/src/geometric/planners/my_planner.rs
use crate::base::planner::Planner;

pub struct MyPlanner<State, Space, Goal> {
    // ... fields
}

impl<State, Space, Goal> Planner<State, Space, Goal> for MyPlanner<State, Space, Goal>
where
    // ... constraints
{
    // ... implementation
}
```

## Python Bindings (`oxmpl-py`)
To make your planner available in Python, you need to wrap the Rust struct using PyO3.

1.  **Create Wrapper File**: Create `oxmpl-py/src/geometric/<my_planner>.rs`.
2.  **Define Type Aliases**: Since PyO3 classes cannot be generic, define type aliases for your planner specialized for each supported state space (e.g., `MyPlannerForRealVector`, `MyPlannerForSO2`, etc.).
3.  **Define the PyO3 Class**: Create a struct (e.g., `PyMyPlanner`) decorated with `#[pyclass]`.
    *   It should hold an enum `PlannerVariant` that wraps the specialized Rust planner instances (usually inside `Rc<RefCell<...>>`).
4.  **Implement `__init__`**: In `#[new]`, inspect the `PyProblemDefinition` to determine which variant to instantiate.
5.  **Implement Methods**: Implement `solve`, `setup`, etc., delegating the call to the inner Rust planner.
6.  **Export**: Register the class in `oxmpl-py/src/geometric/mod.rs`.

**Template:**

```rust,ignore
// oxmpl-py/src/geometric/my_planner.rs
use pyo3::prelude::*;
use crate::base::{ProblemDefinitionVariant, PyGoal, PyPath, PyPlannerConfig, PyProblemDefinition};
use oxmpl::geometric::MyPlanner;
// ... imports

// 1. Define aliases for concrete types
type MyPlannerForRealVector = MyPlanner<RealVectorState, RealVectorStateSpace, PyGoal<RealVectorState>>;
// ... repeat for SO2, SO3, SE2, SE3, Compound

// 2. Define enum wrapper
enum PlannerVariant {
    RealVector(Rc<RefCell<MyPlannerForRealVector>>),
    // ...
}

// 3. Define Python class
#[pyclass(name = "MyPlanner", unsendable)]
pub struct PyMyPlanner {
    planner: PlannerVariant,
    pd: ProblemDefinitionVariant,
}

#[pymethods]
impl PyMyPlanner {
    #[new]
    fn new(
        problem_definition: &PyProblemDefinition,
        planner_config: &PyPlannerConfig,
    ) -> PyResult<Self> {
        // Switch on problem_definition type to create correct PlannerVariant
    }

    fn solve(&mut self, timeout_secs: f32) -> PyResult<PyPath> {
        // Delegate to self.planner
    }
}
```

## JavaScript Bindings (`oxmpl-js`)
To make your planner available in JavaScript/WASM, wrap it using `wasm-bindgen`.

1.  **Create Wrapper File**: Create `oxmpl-js/src/geometric/<my_planner>.rs`.
2.  **Define Enum Wrapper**: Similar to Python, define an enum `MyPlannerVariant` holding the specialized Rust planners.
3.  **Define JS Class**: Create a struct `JsMyPlanner` decorated with `#[wasm_bindgen]`.
4.  **Implement Constructor**: Match the `JsProblemDefinition` variant to instantiate the correct planner type.
5.  **Implement Methods**: Implement `solve` etc., delegating to the inner planner and returning `JsPath`.
6.  **Export**: Add to `oxmpl-js/src/geometric/mod.rs`.

**Template:**

```rust,ignore
// oxmpl-js/src/geometric/my_planner.rs
use wasm_bindgen::prelude::*;
use crate::base::{JsPath, JsProblemDefinition, ProblemDefinitionVariant, JsPlannerConfig};
use oxmpl::geometric::MyPlanner;

enum MyPlannerVariant {
    RealVector(MyPlanner<RealVectorState, RealVectorStateSpace, JsGoal>),
    // ...
}

#[wasm_bindgen(js_name = MyPlanner)]
pub struct JsMyPlanner {
    planner: MyPlannerVariant,
    pd: ProblemDefinitionVariant,
}

#[wasm_bindgen(js_class = MyPlanner)]
impl JsMyPlanner {
    #[wasm_bindgen(constructor)]
    pub fn new(problem_def: &JsProblemDefinition, config: &JsPlannerConfig) -> Self {
        // Switch on problem_def.inner to create correct MyPlannerVariant
    }

    pub fn solve(&mut self, timeout_secs: f32) -> Result<JsPath, String> {
        // Delegate and convert result
    }
}
```

## Integration Testing
Strict testing is required for any new planner to ensure correctness and stability across all bindings. You must implement integration tests for **each layer** of the stack.

### Rust (`oxmpl/tests/`)
Create a new integration test file `oxmpl/tests/<planner_name>_tests.rs`.
*   Verify the planner solves basic problems (e.g., finding a path in a free space).
*   Test with **all supported state spaces** (RealVector, SO2, SO3, SE2, SE3, Compound).
*   Ensure the planner respects time limits and goal conditions.

### Python (`oxmpl-py/tests/`)
Add tests in `oxmpl-py/tests/` using `pytest`.
*   Create a file `test_<planner_name>.py`.
*   Implement tests that mirror the Rust integration tests.
*   Ensure the Python bindings correctly expose the planner and return valid paths.
*   Run tests with: `pytest oxmpl-py/tests/`

### JavaScript (`oxmpl-js/tests/`)
Add tests in `oxmpl-js/tests/` using `vitest`.
*   Create a file `test_<planner_name>.test.js`.
*   Verify the planner works in a simulated JS environment.
*   Check that the API handles JavaScript objects correctly (e.g., passing options, callbacks).
*   Run tests with: `npm test` inside the `oxmpl-js` directory.
