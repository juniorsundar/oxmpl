# Compound State Planning: Rust
This example demonstrates how to use `CompoundStateSpace` to combine multiple state spaces (in this case, `RealVectorStateSpace` for position and `SO2StateSpace` for orientation) into a single planning problem.

```rust,ignore
{{#include ../../../oxmpl/examples/compound_state_planning.rs}}
```
