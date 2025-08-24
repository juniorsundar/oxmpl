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
use oxmpl::geometric::PRM;

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

/// A helper function to validate the entire solution path.
fn is_path_valid(
    path: &Path<RealVectorState>,
    space: &RealVectorStateSpace,
    checker: &dyn StateValidityChecker<RealVectorState>,
) -> bool {
    for i in 0..path.0.len() - 1 {
        let state_a = &path.0[i];
        let state_b = &path.0[i + 1];

        if !checker.is_valid(state_a) {
            println!("Path invalid: State {state_a:?} is in collision.");
            return false;
        }
        if (i + 1 == path.0.len() - 1) && !checker.is_valid(state_b) {
            println!("Path invalid: Final state {state_b:?} is in collision.");
            return false;
        }

        let dist = space.distance(state_a, state_b);
        let num_steps = (dist / 0.1).ceil() as usize;
        if num_steps > 1 {
            let mut interpolated_state = state_a.clone();
            for j in 1..=num_steps {
                let t = j as f64 / num_steps as f64;
                space.interpolate(state_a, state_b, t, &mut interpolated_state);
                if !checker.is_valid(&interpolated_state) {
                    println!(
                        "Path invalid: Motion between {state_a:?} and {state_b:?} is in collision at {interpolated_state:?}."
                    );
                    return false;
                }
            }
        }
    }
    true
}

#[test]
fn test_prm_finds_path_in_rvss() {
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
    // Let's ensure our start/goal are not inside the wall
    assert!(
        validity_checker.is_valid(&start_state),
        "Start state should be valid!"
    );
    assert!(
        validity_checker.is_valid(&goal_definition.target),
        "Goal target should be valid!"
    );

    let mut planner = PRM::new(5.0, 0.5);

    planner.setup(problem_definition, validity_checker.clone());
    match planner.construct_roadmap() {
        Err(_) => panic!("Issue constructing roadmap!"),
        Ok(_) => assert!(
            !planner.get_roadmap().is_empty(),
            "Roadmap was not populated."
        ),
    };

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

    println!("PRM planner test passed!");
}
