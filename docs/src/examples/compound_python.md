# Compound State Planning: Python
This example demonstrates how to use `CompoundStateSpace` — the **dynamic API** for arbitrary ordered products of component spaces — to plan for a system with both position and orientation. For fixed rigid-body configurations like planar or spatial motion, use `SE2StateSpace` or `SE3StateSpace` instead.

```python
{{#include ../../../oxmpl-py/examples/compound_state_planning.py}}
```
