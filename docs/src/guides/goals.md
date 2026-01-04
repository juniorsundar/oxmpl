# Goal Definitions
The `Goal` component defines the termination condition for the planner. It tells the planner when it has successfully found a valid configuration.

## Core Concepts
A Goal generally consists of three parts (though some are optional depending on the planner):

1.  **`is_satisfied(state) -> bool`**: Returns `true` if the state is in the goal region.
2.  **`distance_goal(state) -> float`**: Heuristic distance to the goal. Used by some planners to bias sampling towards the goal.
3.  **`sample_goal() -> State`**: Returns a random state within the goal region. Required by bi-directional planners (like `RRTConnect`) to grow trees from the goal.

## Implementations
### Rust
OxMPL provides traits for different levels of goal capability: `Goal`, `GoalRegion`, and `GoalSampleableRegion`.

```rust
use oxmpl::base::{
    goal::{Goal, GoalRegion, GoalSampleableRegion},
    state::RealVectorState,
    space::RealVectorStateSpace,
    error::StateSamplingError
};
use std::sync::Arc;
use rand::Rng;

struct MyGoal {
    target: RealVectorState,
    threshold: f64,
    space: Arc<RealVectorStateSpace>,
}

// Basic Goal: Just checks satisfaction
impl Goal<RealVectorState> for MyGoal {
    fn is_satisfied(&self, state: &RealVectorState) -> bool {
        self.space.distance(state, &self.target) <= self.threshold
    }
}

// GoalRegion: Provides distance heuristic
impl GoalRegion<RealVectorState> for MyGoal {
    fn distance_goal(&self, state: &RealVectorState) -> f64 {
        let dist = self.space.distance(state, &self.target);
        (dist - self.threshold).max(0.0)
    }
}

// GoalSampleableRegion: Allows sampling (Required for RRTConnect)
impl GoalSampleableRegion<RealVectorState> for MyGoal {
    fn sample_goal(&self, _rng: &mut impl Rng) -> Result<RealVectorState, StateSamplingError> {
        // Simple implementation: just return the target
        // A better implementation would sample around the target
        Ok(self.target.clone())
    }
}
```

### Python
In Python, you define a class that implements the required methods.

```python
import random
from oxmpl_py.base import RealVectorStateSpace, RealVectorState

class MyGoal:
    def __init__(self, space, target_x, target_y, radius):
        self.space = space
        self.target = RealVectorState([target_x, target_y])
        self.radius = radius

    def is_satisfied(self, state: RealVectorState) -> bool:
        return self.space.distance(state, self.target) <= self.radius

    def distance_goal(self, state: RealVectorState) -> float:
        dist = self.space.distance(state, self.target)
        return max(0.0, dist - self.radius)

    def sample_goal(self) -> RealVectorState:
        # Sample a random point within the goal circle
        angle = random.uniform(0, 6.28)
        r = self.radius * random.uniform(0, 1)**0.5
        x = self.target.values[0] + r * math.cos(angle)
        y = self.target.values[1] + r * math.sin(angle)
        return RealVectorState([x, y])

# Usage
goal = MyGoal(space, 5.0, 5.0, 0.5)
problem = ProblemDefinition.from_real_vector(space, start, goal)
```

### JavaScript
In JavaScript, you pass an object (or class instance) with the required function properties.

```javascript
const goal = new oxmpl.base.Goal({
    isSatisfied: (state) => {
        return space.distance(state, targetState) <= radius;
    },

    distanceGoal: (state) => {
        const dist = space.distance(state, targetState);
        return Math.max(0, dist - radius);
    },

    sampleGoal: () => {
        // Return a new State object inside the goal region
        return new oxmpl.base.RealVectorState([5.0, 5.0]);
    }
});
```

## Types of Goals
1.  **Fixed State Goal**: The goal is a single specific configuration.
2.  **Goal Region**: The goal is a set of states.
3.  **Threshold Goal**: The goal is defined by a condition.
