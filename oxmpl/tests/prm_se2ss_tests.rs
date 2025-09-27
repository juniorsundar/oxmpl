use std::{f64::consts::PI, sync::Arc, time::Duration};

use oxmpl::base::{
    error::StateSamplingError,
    goal::{Goal, GoalRegion, GoalSampleableRegion},
    planner::{Path, Planner},
    problem_definition::ProblemDefinition,
    space::{SE2StateSpace, StateSpace},
    state::SE2State,
    validity::StateValidityChecker,
};
use oxmpl::geometric::PRM;

use rand::Rng;

struct ObstacleChecker {
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
}

impl StateValidityChecker<SE2State> for ObstacleChecker {
    fn is_valid(&self, state: &SE2State) -> bool {
        let x = state.get_x();
        let y = state.get_y();
        !(x >= self.x_min && x <= self.x_max && y >= self.y_min && y <= self.y_max)
    }
}

struct SE2GoalRegion {
    target: SE2State,
    radius: f64,
    space: Arc<SE2StateSpace>,
}

impl Goal<SE2State> for SE2GoalRegion {
    fn is_satisfied(&self, state: &SE2State) -> bool {
        self.space.distance(state, &self.target) <= self.radius
    }
}

impl GoalRegion<SE2State> for SE2GoalRegion {
    fn distance_goal(&self, state: &SE2State) -> f64 {
        let dist_to_center = self.space.distance(state, &self.target);
        (dist_to_center - self.radius).max(0.0)
    }
}

impl GoalSampleableRegion<SE2State> for SE2GoalRegion {
    fn sample_goal(&self, rng: &mut impl Rng) -> Result<SE2State, StateSamplingError> {
        let angle = rng.random_range(0.0..2.0 * PI);
        let r = self.radius * rng.random::<f64>().sqrt();
        let x = self.target.get_x() + r * angle.cos();
        let y = self.target.get_y() + r * angle.sin();
        let yaw = rng.random_range(-PI..PI);
        Ok(SE2State::new(x, y, yaw))
    }
}

fn is_path_valid(
    path: &Path<SE2State>,
    space: &SE2StateSpace,
    checker: &dyn StateValidityChecker<SE2State>,
) -> bool {
    for window in path.0.windows(2) {
        let state_a = &window[0];
        let state_b = &window[1];

        if !checker.is_valid(state_a) {
            println!(
                "Path invalid: State (x: {}, y: {}, yaw: {}) is in collision.",
                state_a.get_x(),
                state_a.get_y(),
                state_a.get_yaw()
            );
            return false;
        }

        let dist = space.distance(state_a, state_b);
        let num_steps = (dist / space.get_longest_valid_segment_length()).ceil() as usize;
        if num_steps > 1 {
            let mut interpolated_state = state_a.clone();
            for j in 1..=num_steps {
                let t = j as f64 / num_steps as f64;
                space.interpolate(state_a, state_b, t, &mut interpolated_state);
                if !checker.is_valid(&interpolated_state) {
                    println!(
                        "Path invalid: Motion between (x: {}, y: {}, yaw: {}) and (x: {}, y: {}, yaw: {}) is in collision at (x: {}, y: {}, yaw: {}).",
                        state_a.get_x(), state_a.get_y(), state_a.get_yaw(),
                        state_b.get_x(), state_b.get_y(), state_b.get_yaw(),
                        interpolated_state.get_x(), interpolated_state.get_y(), interpolated_state.get_yaw()
                    );
                    return false;
                }
            }
        }
    }
    if let Some(last_state) = path.0.last() {
        if !checker.is_valid(last_state) {
            println!(
                "Path invalid: Final state (x: {}, y: {}, yaw: {}) is in collision.",
                last_state.get_x(),
                last_state.get_y(),
                last_state.get_yaw()
            );
            return false;
        }
    }
    true
}

#[test]
#[allow(clippy::arc_with_non_send_sync)]
fn test_prm_finds_path_in_se2ss() {
    let bounds = vec![(-5.0, 5.0), (-5.0, 5.0), (-PI, PI)];
    let space =
        Arc::new(SE2StateSpace::new(0.5, Some(bounds)).expect("Failed to create state space"));

    let start_state = SE2State::new(-2.0, 0.0, 0.0);
    let goal_definition = Arc::new(SE2GoalRegion {
        target: SE2State::new(2.0, 0.0, 0.0),
        radius: 0.5,
        space: space.clone(),
    });

    let problem_definition = Arc::new(ProblemDefinition {
        space: space.clone(),
        start_states: vec![start_state.clone()],
        goal: goal_definition.clone(),
    });

    let validity_checker = Arc::new(ObstacleChecker {
        x_min: -0.25,
        x_max: 0.25,
        y_min: -0.5,
        y_max: 0.5,
    });
    assert!(
        validity_checker.is_valid(&start_state),
        "Start state should be valid!"
    );
    assert!(
        validity_checker.is_valid(&goal_definition.target),
        "Goal target should be valid!"
    );

    let mut planner = PRM::new(40.0, 5.0);

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
