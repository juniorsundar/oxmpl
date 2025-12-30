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
use oxmpl::geometric::PRM;

use rand::Rng;
use std::f64::consts::PI;

struct ObstacleChecker;

impl StateValidityChecker<SE3State> for ObstacleChecker {
    fn is_valid(&self, state: &SE3State) -> bool {
        let x = state.get_x();
        let y = state.get_y();
        let z = state.get_z();

        let obs1_x = 3.0;
        let obs1_y = 3.0;
        let obs1_z = 3.0;
        let obs1_rad = 1.0;

        let dist_sq = (x - obs1_x).powi(2) + (y - obs1_y).powi(2) + (z - obs1_z).powi(2);
        if dist_sq.sqrt() < obs1_rad {
            return false;
        }

        let obs2_x_min = 6.0;
        let obs2_x_max = 8.0;
        let obs2_y_min = 1.0;
        let obs2_y_max = 4.0;
        let obs2_z_min = 2.0;
        let obs2_z_max = 5.0;

        if x >= obs2_x_min
            && x <= obs2_x_max
            && y >= obs2_y_min
            && y <= obs2_y_max
            && z >= obs2_z_min
            && z <= obs2_z_max
        {
            return false;
        }

        true
    }
}

struct SE3GoalRegion {
    target: SE3State,
    radius: f64,
    _space: Arc<SE3StateSpace>,
}

impl Goal<SE3State> for SE3GoalRegion {
    fn is_satisfied(&self, state: &SE3State) -> bool {
        let dx = state.get_x() - self.target.get_x();
        let dy = state.get_y() - self.target.get_y();
        let dz = state.get_z() - self.target.get_z();
        (dx * dx + dy * dy + dz * dz).sqrt() <= self.radius
    }
}

impl GoalRegion<SE3State> for SE3GoalRegion {
    fn distance_goal(&self, state: &SE3State) -> f64 {
        let dx = state.get_x() - self.target.get_x();
        let dy = state.get_y() - self.target.get_y();
        let dz = state.get_z() - self.target.get_z();
        let dist = (dx * dx + dy * dy + dz * dz).sqrt();
        (dist - self.radius).max(0.0)
    }
}

impl GoalSampleableRegion<SE3State> for SE3GoalRegion {
    fn sample_goal(&self, rng: &mut impl Rng) -> Result<SE3State, StateSamplingError> {
        let u: f64 = rng.random();
        let v: f64 = rng.random();
        let theta = 2.0 * PI * u;
        let phi = (2.0 * v - 1.0).acos();
        let r = self.radius * rng.random::<f64>().cbrt();

        let x = self.target.get_x() + r * phi.sin() * theta.cos();
        let y = self.target.get_y() + r * phi.sin() * theta.sin();
        let z = self.target.get_z() + r * phi.cos();

        let quat_w: f64 = rng.random_range(-1.0..1.0);
        let quat_x: f64 = rng.random_range(-1.0..1.0);
        let quat_y: f64 = rng.random_range(-1.0..1.0);
        let quat_z: f64 = rng.random_range(-1.0..1.0);
        let norm = (quat_w.powi(2) + quat_x.powi(2) + quat_y.powi(2) + quat_z.powi(2)).sqrt();

        let rotation = if norm > 1e-9 {
            SO3State::new(quat_x / norm, quat_y / norm, quat_z / norm, quat_w / norm)
        } else {
            SO3State::identity()
        };

        Ok(SE3State::new(x, y, z, rotation))
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
fn test_prm_finds_path_in_se3ss() {
    let bounds = vec![(0.0, 10.0), (0.0, 10.0), (0.0, 10.0)];
    let space =
        Arc::new(SE3StateSpace::new(1.0, Some(bounds)).expect("Failed to create SE3 state space"));

    let start_state = SE3State::new(1.0, 1.0, 1.0, SO3State::new(0.0, 0.0, 0.0, 1.0));
    let goal_definition = Arc::new(SE3GoalRegion {
        target: SE3State::new(9.0, 9.0, 9.0, SO3State::identity()),
        radius: 0.5,
        _space: space.clone(),
    });

    let problem_definition = Arc::new(ProblemDefinition {
        space: space.clone(),
        start_states: vec![start_state.clone()],
        goal: goal_definition.clone(),
    });

    let validity_checker = Arc::new(ObstacleChecker);

    assert!(
        validity_checker.is_valid(&start_state),
        "Start state must be valid"
    );
    assert!(
        validity_checker.is_valid(&goal_definition.target),
        "Goal target must be valid"
    );

    let mut planner = PRM::new(5.0, 2.0, &PlannerConfig { seed: Some(0) });
    planner.setup(problem_definition, validity_checker.clone());

    planner
        .construct_roadmap()
        .expect("Roadmap construction failed");

    let result = planner.solve(Duration::from_secs(5));
    assert!(result.is_ok(), "Failed to find solution path");

    let path = result.unwrap();
    assert!(!path.0.is_empty());
    assert!(space.distance(path.0.first().unwrap(), &start_state) < 1e-9);
    assert!(goal_definition.is_satisfied(path.0.last().unwrap()));
    assert!(is_path_valid(&path, &space, &*validity_checker));

    println!("PRM SE3 test passed!");
}
