# Quick Start: Python

Here is a complete example of solving a 2D planning problem with a custom collision checker written in Python.

```python
import math
from oxmpl_py.base import RealVectorState, RealVectorStateSpace, ProblemDefinition, PlannerConfig
from oxmpl_py.geometric import RRT

def is_state_valid(state: RealVectorState) -> bool:
    """A state is invalid if it's inside a circular obstacle at the origin."""
    x, y = state.values
    return math.sqrt(x**2 + y**2) > 2.0

class MyCircularGoal:
    def __init__(self, space, x, y, radius):
        self.space = space
        self.target = RealVectorState([x, y])
        self.radius = radius

    def is_satisfied(self, state: RealVectorState) -> bool:
        return self.space.distance(self.target, state) <= self.radius

    def sample_goal(self) -> RealVectorState:
        return self.target

space = RealVectorStateSpace(dimension=2, bounds=[(-10.0, 10.0), (-10.0, 10.0)])
start_state = RealVectorState([-5.0, -5.0])
goal_region = MyCircularGoal(space, x=5.0, y=5.0, radius=0.5)

problem_def = ProblemDefinition.from_real_vector(space, start_state, goal_region)
planner_config = PlannerConfig(seed=123)

planner = RRT(max_distance=0.5,
        goal_bias=0.05,
        problem_definition=problem_def,
        planner_config=planner_config)
planner.setup(is_state_valid)

try:
    path = planner.solve(timeout_secs=5.0)
    print(f"Solution found with {len(path.states)} states!")
except Exception as e:
    print(f"Planning failed: {e}")
```
