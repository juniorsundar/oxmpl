# State Validity Checkers
The `StateValidityChecker` is a crucial component in any motion planning problem. It defines all the valide configurations the robot is allowed to be. Its primary job is to take a state and return `true` if it is valid (collision-free, within constraints) or `false` otherwise.

## The Interface
A validity checker is essentially a function or closure with the signature:
`fn(state) -> bool`

Since planners call this function thousands of times, it **must be efficient**.

## Implementations
### Rust
In Rust, you implement the `StateValidityChecker` trait for your specific state type.

```rust
use oxmpl::base::{validity::StateValidityChecker, state::RealVectorState};

struct MyObstacleChecker {
    // Store efficient data structures here
    obstacles: Vec<(f64, f64, f64)>, // x, y, radius
}

impl StateValidityChecker<RealVectorState> for MyObstacleChecker {
    fn is_valid(&self, state: &RealVectorState) -> bool {
        let x = state.values[0];
        let y = state.values[1];

        for (ox, oy, r) in &self.obstacles {
            let dist_sq = (x - ox).powi(2) + (y - oy).powi(2);
            if dist_sq < r.powi(2) {
                return false; // Collision!
            }
        }
        true
    }
}
```

### Python
In Python, you simply provide a callable (function or lambda) that accepts a state and returns a boolean.

```python
from oxmpl_py.base import RealVectorState

def is_state_valid(state: RealVectorState) -> bool:
    x, y = state.values

    # Check bounds (optional if StateSpace already handles it, but good practice)
    if not (-10 < x < 10 and -10 < y < 10):
        return False

    # Check circular obstacle at (0,0) with radius 2
    if x*x + y*y < 4.0:
        return False

    return True

# Pass this function to the planner setup
planner.setup(is_state_valid)
```

### JavaScript
In JavaScript, you pass a callback function to the `StateValidityChecker` constructor.

```javascript
const validityChecker = new oxmpl.base.StateValidityChecker((state) => {
    // Assuming RealVectorState
    const [x, y] = state.values;

    // Check circular obstacle
    if (x*x + y*y < 4.0) {
        return false;
    }

    return true;
});

planner.setup(validityChecker);
```

## Best Practices
1.  **Fail Fast**: Check the most likely collisions first. If you have a bounding box for your robot, check that against the environment bounds before doing complex mesh collision detection.
2.  **Broad-Phase vs. Narrow-Phase**: Use simple geometric shapes (spheres, boxes) to approximate obstacles for a quick check. Only perform expensive checks (detailed mesh intersection) if the broad-phase check passes.
3.  **Thread Safety**: The `StateValidityChecker` trait enforces `Send + Sync`. This is in anticipation of a future where multithreading will be implemented.
