# Quick Start: Rust

Here is a complete example of solving a 2D planning problem with a custom collision checker and goal region.

```rust,ignore
use std::{f64::consts::PI, sync::Arc, time::Duration};

use oxmpl::base::{
    error::StateSamplingError,
    goal::{Goal, GoalRegion, GoalSampleableRegion},
    planner::{Path, Planner, PlannerConfig},
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

    let space = Arc::new(new_rvss_result.expect("Error creating new RealVectorState!"));

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

    let mut planner = RRT::new(0.5, 0.0, &PlannerConfig{ seed: Some(123) });

    planner.setup(problem_definition, validity_checker.clone());

    let timeout = Duration::from_secs(5);
    let result = planner.solve(timeout);

    let path = result.expect("Planner failed to find a solution");
    println!("Found path with {} states.", path.0.len());
}
```
