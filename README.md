<div align="center">

# Ox[MPL](https://github.com/ompl/ompl)

The Open Motion-Planning Library but Oxidised

</div>
<div align="center">
<br>
</div>

# Additional Links

- [Backlog](./docs/BACKLOG.md)

# Introduction

## The What?

`oxmpl` is a sampling-based motion planning library written in Rust, inspired by the structure and concepts of the [Open Motion Planning Library (OMPL)](http://ompl.kavrakilab.org/).

It is **NOT** OMPL with Rust bindings.

It provides a flexible and type-safe Rust API for core planning algorithms and offers high-level Python bindings for rapid prototyping and integration into existing Python-based robotics projects.

## The Why?

> [!WARNING]
> *I will delete this section when the project hits v1.0.0 or when I decide to create a dedicated documentation static site.*

OMPL is great, but it isn't written in Rust. While that's not a valid reason to rewrite it, the truth is C++ doesn't particularly *spark joy* for me. Nonetheless, it's a library I had to use regularly as a university student and still use in my work.

I'm teaching myself Rust, and after a few small projects, I found that it *sparked joy*. So, to really dive deep into the language, I decided to do what many Rustaceans do: rewrite something that already exists. In this case, my target was OMPL.

My goal isn't to create a full "drop-in" replacement for OMPL. I've found that Rust's traits and implementations can handle OMPL's modular nature more elegantly. However, I do want to provide Python bindings so the library can be used by people not keen on diving into Rust (looking at you, researchers).

This library will be similar to OMPL at a high level since I'm using the original as a reference. I'm still relatively new to Rust, so I'll often refer to the C++ source to help structure the code.

If you spot any issues or have ideas for improvements, please don't hesitate to open an issue or a PR. It would be a great learning opportunity for me.

# Key Features

* **Safe & Fast**: Built in Rust for memory safety and performance.
* **Extensible Architecture**: Core concepts like `StateSpace`, `StateValidityChecker`, and `Planner` are defined as traits, making the library highly extensible.
* **Pythonic Bindings**: A simple, easy-to-use Python API powered by PyO3 that allows users to define problem-specific logic (like collision checkers) in pure Python.
* **Inspired by OMPL**: Follows the modular design of OMPL, making it familiar to those in the robotics community.

# Installation

You can use `oxmpl` in Rust, Python, or JavaScript projects.

## **For Python Users**

The library is available on PyPI and can be installed with `pip`:

```bash
pip install oxmpl-py
```

## **For JavaScript/TypeScript Users**

JavaScript/WASM bindings are available:

```bash
npm install oxmpl-js  # Coming soon
```

> **Note:** Currently available as source. See [oxmpl-js/README.md](./oxmpl-js/README.md) for build instructions.

## For Rust Users

The core library is available on crates.io and can be added to your project's `Cargo.toml`:
```toml
[dependencies]
oxmpl = "0.2.0" # Replace with the latest version
```

# Quick Start

## JavaScript

```javascript
import * as oxmpl from 'oxmpl-js';

// A state is invalid if it's inside a circular obstacle at the origin
function isStateValid(state) {
  const [x, y] = state;
  return Math.sqrt(x * x + y * y) > 2.0;
}

// Create a 2D state space with bounds
const space = new oxmpl.RealVectorStateSpace(2, [-10.0, 10.0, -10.0, 10.0]);

// Define start state
const start = new Float64Array([-5.0, -5.0]);

// Define circular goal region
const target = [5.0, 5.0];
const radius = 0.5;
const goal = new oxmpl.Goal(
  (state) => {
    const [x, y] = state;
    const dist = Math.sqrt((x - target[0]) ** 2 + (y - target[1]) ** 2);
    return dist <= radius;
  },
  (state) => {
    const [x, y] = state;
    const dist = Math.sqrt((x - target[0]) ** 2 + (y - target[1]) ** 2);
    return Math.max(0, dist - radius);
  },
  () => new Float64Array(target) // Sample goal center
);

// Create problem and run planner
const problem = new oxmpl.ProblemDefinition(space, start, goal);
const validityChecker = new oxmpl.StateValidityChecker(isStateValid);

const planner = new oxmpl.RRT(0.5, 0.05);
planner.setup(problem, validityChecker);

try {
  const path = planner.solve(5.0);
  if (path && path.length() > 0) {
    console.log(`Solution found with ${path.length()} states!`);
  } else {
    console.log('No solution found');
  }
} catch (e) {
  console.log(`Planning failed: ${e}`);
}
```

## Python

Here is a complete example of solving a 2D planning problem with a custom collision checker written in Python.

```python
import math
from oxmpl_py.base import RealVectorState, RealVectorStateSpace, ProblemDefinition
from oxmpl_py.geometric.planners import RRT

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
        # A real implementation would use a random number generator
        return self.target

space = RealVectorStateSpace(dimension=2, bounds=[(-10.0, 10.0), (-10.0, 10.0)])
start_state = RealVectorState([-5.0, -5.0])
goal_region = MyCircularGoal(space, x=5.0, y=5.0, radius=0.5)

problem_def = ProblemDefinition.from_real_vector(space, start_state, goal_region)

planner = RRT(max_distance=0.5, goal_bias=0.05, problem_definition=problem_def)
planner.setup(is_state_valid)

try:
    path = planner.solve(timeout_secs=5.0)
    print(f"Solution found with {len(path.states)} states!")
    # for state in path.states:
    #     print(f"  -> {state.values}")
except Exception as e:
    print(f"Planning failed: {e}")
```

## Rust

```rust
use std::{f64::consts::PI, sync::Arc, time::Duration};

use oxmpl::base::{
    error::StateSamplingError,
    goal::{Goal, GoalRegion, GoalSampleableRegion},
    planner::{Path, Planner},
    problem_definition::ProblemDefinition,
    space::{RealVectorStateSpace, StateSpace},
    state::RealVectorState,
    validity::StateValidityChecker,
};
use oxmpl::geometric::planners::rrt::RRT;

use rand::Rng;

/// A StateValidityChecker that defines a simple vertical wall obstacle.
struct WallObstacleChecker {
    wall_x_pos: f64,
    wall_y_min: f64,
    wall_y_max: f64,
    wall_thickness: f64,
}

impl StateValidityChecker<RealVectorState> for WallObstacleChecker {
    fn is_valid(&self, state: &RealVectorState) -> bool {
        let x = state.values[0];
        let y = state.values[1];

        let is_in_wall = x >= self.wall_x_pos - self.wall_thickness / 2.0
            && x <= self.wall_x_pos + self.wall_thickness / 2.0
            && y >= self.wall_y_min
            && y <= self.wall_y_max;

        !is_in_wall
    }
}

/// A Goal definition where success is being within a certain radius of a target state.
struct CircularGoalRegion {
    target: RealVectorState,
    radius: f64,
    space: Arc<RealVectorStateSpace>,
}

impl Goal<RealVectorState> for CircularGoalRegion {
    fn is_satisfied(&self, state: &RealVectorState) -> bool {
        self.space.distance(state, &self.target) <= self.radius
    }
}

impl GoalRegion<RealVectorState> for CircularGoalRegion {
    fn distance_goal(&self, state: &RealVectorState) -> f64 {
        let dist_to_center = self.space.distance(state, &self.target);
        (dist_to_center - self.radius).max(0.0)
    }
}

impl GoalSampleableRegion<RealVectorState> for CircularGoalRegion {
    fn sample_goal(&self, rng: &mut impl Rng) -> Result<RealVectorState, StateSamplingError> {
        let angle = rng.random_range(0.0..2.0 * PI);

        let radius = self.radius * rng.random::<f64>().sqrt();

        let x = self.target.values[0] + radius * angle.cos();
        let y = self.target.values[1] + radius * angle.sin();

        Ok(RealVectorState { values: vec![x, y] })
    }
}

fn main() {
    let new_rvss_result = RealVectorStateSpace::new(2, Some(vec![(0.0, 10.0), (0.0, 10.0)]));

    let space;
    match new_rvss_result {
        Ok(state) => space = Arc::new(state),
        Err(_) => {
            panic!("Error creating new RealVectorState!")
        }
    }

    let start_state = RealVectorState {
        values: vec![1.0, 5.0],
    };
    let goal_definition = Arc::new(CircularGoalRegion {
        target: RealVectorState {
            values: vec![9.0, 5.0],
        },
        radius: 0.5,
        space: space.clone(),
    });

    let problem_definition = Arc::new(ProblemDefinition {
        space: space.clone(),
        start_states: vec![start_state.clone()],
        goal: goal_definition.clone(),
    });

    let validity_checker = Arc::new(WallObstacleChecker {
        wall_x_pos: 5.0,
        wall_y_min: 2.0,
        wall_y_max: 8.0,
        wall_thickness: 0.5,
    });

    let mut planner = RRT::new(0.5, 0.0);

    planner.setup(problem_definition, validity_checker.clone());

    let timeout = Duration::from_secs(5);
    let result = planner.solve(timeout);

    assert!(
        result.is_ok(),
        "Planner failed to find a solution when one should exist. Error: {:?}",
        result.err()
    );

    let path = result.unwrap();
    println!("Found path with {} states.", path.0.len());

    assert!(!path.0.is_empty(), "Path should not be empty");

    assert!(
        space.distance(path.0.first().unwrap(), &start_state) < 1e-9,
        "Path should start at the start state"
    );

    assert!(
        goal_definition.is_satisfied(path.0.last().unwrap()),
        "Path should end in the goal region"
    );

    assert!(
        is_path_valid(&path, &space, &*validity_checker),
        "The returned path was found to be invalid."
    );
}

```

# Project Structure

This project is a Cargo workspace containing three separate crates:

* **oxmpl/**: The core Rust library containing all the planning logic, traits, and data structures. It has no Python or JavaScript-specific code.
* **oxmpl-py/**: A lightweight crate that contains the PyO3 bindings to expose the functionality of the `oxmpl` library to Python.
* **oxmpl-js/**: JavaScript/WASM bindings using wasm-pack to provide `oxmpl` functionality in web browsers and Node.js.

# Development

## Prerequisites

* Rust toolchain (via rustup)
* Python 3.8+ and a virtual environment
* `maturin` for building the Python bindings

## Setup

```bash
# Clone the repository
git clone https://github.com/juniorsundar/oxmpl.git
cd oxmpl

# Set up the Python environment (optional, but recommended)
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt # Installs maturin, pytest, etc.
```

## Running Tests

To run the complete test suite for both crates:
```bash
# Run all Rust unit and integration tests
cargo test --all

# Run the Python integration tests
# (make sure the module is installed in your venv first)
maturin develop
pytest
```

# Contributing

Contributions are welcome! Please feel free to open an issue to discuss a bug or new feature, or submit a pull request.

# License

This project is licensed under the **BSD-3-Clause License**. See the [LICENSE](http://docs.google.com/LICENSE) file for details.
