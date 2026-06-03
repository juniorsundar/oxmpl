# Compound State Planning: JavaScript
This example demonstrates how to use `CompoundStateSpace` and `CompoundStateBuilder` — the **dynamic API** for arbitrary ordered products of component spaces — in JavaScript/WASM to solve a planning problem in a composite state space. For fixed rigid-body configurations like planar or spatial motion, use `SE2StateSpace` or `SE3StateSpace` instead.

```javascript
{{#include ../../../oxmpl-js/examples/compound_state_planning.js}}
```
