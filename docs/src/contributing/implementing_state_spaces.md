# Implementing New State Spaces
This guide explains how to add a new `State` and `StateSpace` to OxMPL and expose them to the bindings.

## Do You Need a New State Type?
Before implementing a custom `State` and `StateSpace`, consider if your requirements can be met using `CompoundState` and `CompoundStateSpace`.

### Using `CompoundStateSpace`
Most robotic systems can be represented as a collection of simpler state spaces. A `CompoundStateSpace` allows you to combine existing spaces (like `RealVectorStateSpace`, `SO2StateSpace`, `SO3StateSpace`) via a Cartesian product.

**When to use `CompoundStateSpace`:**
*   Your system is composed of multiple independent parts (e.g., a mobile base ($SE(2)$) + a robotic arm ($R^n$)).
*   You need to assign different weights to different components during distance calculation.

**When to implement a New State:**
*   **Unique Topology**: You are working with a manifold that is not easily represented by existing spaces (e.g., a specific non-Euclidean surface).
*   **Custom Metric**: You require a specialized distance function that cannot be decomposed into component-wise distances (e.g., Dubins path distance for non-holonomic vehicles).
*   **Custom Interpolation**: The path between two states follows specific rules (e.g., geodesic on a sphere).

## Rust Implementation (`oxmpl`)
1.  **Define State**: Create a struct implementing `oxmpl::base::state::State` (and `Clone`, `Debug`, `serde::Serialize`, `serde::Deserialize`).
2.  **Define State Space**: Create a struct implementing `oxmpl::base::space::StateSpace`.
    *   `sample_uniform`: Generate a random state.
    *   `distance`: Compute distance between two states.
    *   `interpolate`: Interpolate between two states.
    *   `satisfies_bounds` / `enforce_bounds`: Check/enforce constraints.
3.  **Export**: Add them to `oxmpl/src/base/states/` and `oxmpl/src/base/spaces/`.

## Python Bindings (`oxmpl-py`)
1.  **Wrap the State**:
    *   Create `oxmpl-py/src/base/<my_state>.rs`.
    *   Define `PyMyState(pub Arc<OxmplMyState>)` with `#[pyclass]`.
    *   Implement `__init__`, getters, and `__repr__`.
    *   Implement `oxmpl_py::base::py_state_convert::PyStateConvert` for `OxmplMyState` to handle conversion between Rust and Python wrappers.
2.  **Wrap the State Space**:
    *   Create `oxmpl-py/src/base/<my_state_space>.rs`.
    *   Define `PyMyStateSpace(pub Arc<Mutex<OxmplMyStateSpace>>)` with `#[pyclass]`.
    *   Implement `distance`, `sample`, etc.
3.  **Register**: Add the classes to `oxmpl-py/src/base/mod.rs` in `create_module`.
4.  **Update Planners**: You may need to update the `PlannerVariant` enums in `oxmpl-py/src/geometric/*.rs` to include a variant for your new state type if it's not covered by existing generic logic.

## JavaScript Bindings (`oxmpl-js`)
1.  **Wrap the State**:
    *   Create `oxmpl-js/src/base/<my_state>.rs`.
    *   Define `JsMyState` struct wrapping `Arc<OxmplMyState>` with `#[wasm_bindgen]`.
    *   Implement constructor and getters.
    *   Implement `oxmpl_js::base::js_state_convert::JsStateConvert` for `OxmplMyState`.
2.  **Wrap the State Space**:
    *   Create `oxmpl-js/src/base/<my_state_space>.rs`.
    *   Define `JsMyStateSpace` struct wrapping `Arc<Mutex<OxmplMyStateSpace>>`.
    *   Implement `sample`, `distance`, etc.
3.  **Register**: Export them in `oxmpl-js/src/base/mod.rs`.
4.  **Update Planners**: Similar to Python, update `oxmpl-js/src/geometric/*.rs` to include the new state variant in the planner enums.

**Note on `JsStateConvert`:**
When implementing `from_js_value` for your state, you often need to inspect the JS object properties to ensure it matches your state type (e.g., check for `x`, `y` for a 2D point).

## Integration Testing
Ensure your new state space is robust by adding tests across the stack.

### Rust
*   **Unit Tests**: Add `#[test]` functions in your implementation file (or a separate test module) to verify:
    *   Distance calculation properties (symmetry, non-negativity).
    *   Interpolation accuracy.
    *   Bounds enforcement.
    *   Serialization/Deserialization (if applicable).
*   **Integration Tests**: Add a test file in `oxmpl/tests/` to check interaction with planners.

### Python (`oxmpl-py/tests/`)
Add tests in `oxmpl-py/tests/` to ensure the bindings are correct.
*   Verify that `__init__` works with valid arguments and raises errors for invalid ones.
*   Check that methods like `distance()` and `sample()` return expected types and values.
*   Ensure that states can be passed to and retrieved from planners correctly.

### JavaScript (`oxmpl-js/tests/`)
Add tests in `oxmpl-js/tests/` to ensure the WASM bindings work as expected.
*   Verify object creation and property access.
*   Test that the `JsStateConvert` trait correctly handles conversion between JS objects and Rust structs.
*   Ensure usage within a `ProblemDefinition` works without panic.
