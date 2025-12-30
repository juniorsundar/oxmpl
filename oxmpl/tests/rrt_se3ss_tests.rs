use std::{sync::Arc, time::Duration};

use oxmpl::base::{
    error::StateSamplingError,
    goal::{Goal, GoalRegion, GoalSampleableRegion},
    planner::{Path, Planner, PlannerConfig},
    problem_definition::ProblemDefinition,
    space::{SE3StateSpace, StateSpace},
    state::{SE3State, SO3State},
    validity::StateValidityChecker,
};
use oxmpl::geometric::RRT;

use rand::Rng;

struct BoxObstacleChecker {
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    z_min: f64,
    z_max: f64,
}

impl StateValidityChecker<SE3State> for BoxObstacleChecker {
    fn is_valid(&self, state: &SE3State) -> bool {
        let x = state.get_x();
        let y = state.get_y();
        let z = state.get_z();
        !(x >= self.x_min
            && x <= self.x_max
            && y >= self.y_min
            && y <= self.y_max
            && z >= self.z_min
            && z <= self.z_max)
    }
}

struct SE3GoalRegion {
    target: SE3State,
    radius: f64,
    space: Arc<SE3StateSpace>,
}

impl Goal<SE3State> for SE3GoalRegion {
    fn is_satisfied(&self, state: &SE3State) -> bool {
        self.space.distance(state, &self.target) <= self.radius
    }
}

impl GoalRegion<SE3State> for SE3GoalRegion {
    fn distance_goal(&self, state: &SE3State) -> f64 {
        let dist_to_center = self.space.distance(state, &self.target);
        (dist_to_center - self.radius).max(0.0)
    }
}

impl GoalSampleableRegion<SE3State> for SE3GoalRegion {
    fn sample_goal(&self, _rng: &mut impl Rng) -> Result<SE3State, StateSamplingError> {
        let t = &self.target;
        Ok(SE3State::new(
            t.get_x(),
            t.get_y(),
            t.get_z(),
            t.get_rotation().clone(),
        ))
    }
}

fn is_path_valid(
    path: &Path<SE3State>,
    space: &SE3StateSpace,
    checker: &dyn StateValidityChecker<SE3State>,
) -> bool {
    for window in path.0.windows(2) {
        let state_a = &window[0];
        let state_b = &window[1];

        if !checker.is_valid(state_a) {
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
                    return false;
                }
            }
        }
    }
    if let Some(last_state) = path.0.last() {
        if !checker.is_valid(last_state) {
            return false;
        }
    }
    true
}

#[test]
#[allow(clippy::arc_with_non_send_sync)]
fn test_rrt_finds_path_in_se3ss() {
    let bounds = vec![(-10.0, 10.0), (-10.0, 10.0), (-10.0, 10.0)];
    let space =
        Arc::new(SE3StateSpace::new(1.0, Some(bounds)).expect("Failed to create SE3 state space"));

    let start_state = SE3State::new(-5.0, 0.0, 0.0, SO3State::identity());
    let goal_definition = Arc::new(SE3GoalRegion {
        target: SE3State::new(5.0, 0.0, 0.0, SO3State::identity()),
        radius: 0.5,
        space: space.clone(),
    });

    let problem_definition = Arc::new(ProblemDefinition {
        space: space.clone(),
        start_states: vec![start_state.clone()],
        goal: goal_definition.clone(),
    });

    let validity_checker = Arc::new(BoxObstacleChecker {
        x_min: -1.0,
        x_max: 1.0,
        y_min: -1.0,
        y_max: 1.0,
        z_min: -1.0,
        z_max: 1.0,
    });

    assert!(
        validity_checker.is_valid(&start_state),
        "Start state must be valid"
    );
    assert!(
        validity_checker.is_valid(&goal_definition.target),
        "Goal target must be valid"
    );

    let mut planner = RRT::new(1.0, 0.1, &PlannerConfig { seed: Some(0) });
    planner.setup(problem_definition, validity_checker.clone());

    let result = planner.solve(Duration::from_secs(5));
    assert!(result.is_ok(), "Failed to find solution path");

    let path = result.unwrap();
    assert!(!path.0.is_empty());
    assert!(space.distance(path.0.first().unwrap(), &start_state) < 1e-9);
    assert!(goal_definition.is_satisfied(path.0.last().unwrap()));
    assert!(is_path_valid(&path, &space, &*validity_checker));

    println!("RRT SE3 test passed!");
}
