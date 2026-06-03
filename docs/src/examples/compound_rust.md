# Compound State Planning: Rust
This example demonstrates how to use `CompoundStateSpace` — the **dynamic API** for arbitrary ordered products of component spaces — to combine multiple state spaces (in this case, `RealVectorStateSpace` for position and `SO2StateSpace` for orientation) into a single planning problem. For fixed rigid-body configurations like planar or spatial motion, use `SE2StateSpace` or `SE3StateSpace` instead.

```rust,ignore
{{#include ../../../oxmpl/examples/compound_state_planning.rs}}
```
