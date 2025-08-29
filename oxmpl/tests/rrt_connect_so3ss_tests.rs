use std::{f64::consts::PI, sync::Arc, time::Duration};

use oxmpl::base::{
    error::StateSamplingError,
    goal::{Goal, GoalRegion, GoalSampleableRegion},
    planner::{Path, Planner},
    problem_definition::ProblemDefinition,
    space::{SO3StateSpace, StateSpace},
    state::SO3State,
    validity::StateValidityChecker,
};
use oxmpl::geometric::RRTConnect;

use rand::Rng;

/// Utility function to create Quaternions
fn quaternion_from_axis_angle(axis: [f64; 3], angle: f64) -> SO3State {
    let norm = (axis[0].powi(2) + axis[1].powi(2) + axis[2].powi(2)).sqrt();

    if norm < 1e-9 {
        return SO3State {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        };
    }

    let ux = axis[0] / norm;
    let uy = axis[1] / norm;
    let uz = axis[2] / norm;

    let half_angle = angle * 0.5;
    let s = half_angle.sin();
    let c = half_angle.cos();

    SO3State {
        x: ux * s,
        y: uy * s,
        z: uz * s,
        w: c,
    }
}

/// A StateValidityChecker that defines a simple vertical wall obstacle.
struct ForbiddenConeChecker {
    center: SO3State,
    radius: f64,
    space: Arc<SO3StateSpace>,
}

impl StateValidityChecker<SO3State> for ForbiddenConeChecker {
    fn is_valid(&self, state: &SO3State) -> bool {
        self.space.distance(&self.center, state) > self.radius
    }
}

struct SO3GoalRegion {
    target: SO3State,
    radius: f64,
    space: Arc<SO3StateSpace>,
}

impl Goal<SO3State> for SO3GoalRegion {
    fn is_satisfied(&self, state: &SO3State) -> bool {
        self.space.distance(state, &self.target) <= self.radius
    }
}

impl GoalRegion<SO3State> for SO3GoalRegion {
    fn distance_goal(&self, state: &SO3State) -> f64 {
        let dist_to_center = self.space.distance(state, &self.target);
        (dist_to_center - self.radius).max(0.0)
    }
}

impl GoalSampleableRegion<SO3State> for SO3GoalRegion {
    /// Samples a state uniformly from within the goal's cone of freedom.
    fn sample_goal(&self, rng: &mut impl Rng) -> Result<SO3State, StateSamplingError> {
        loop {
            let x: f64 = rng.random_range(-1.0..1.0);
            let y: f64 = rng.random_range(-1.0..1.0);
            let z: f64 = rng.random_range(-1.0..1.0);
            let w: f64 = rng.random_range(-1.0..1.0);

            let norm_sq = x * x + y * y + z * z + w * w;

            if norm_sq > 1e-9 && norm_sq < 1.0 {
                let norm = norm_sq.sqrt();
                let random_quat = SO3State {
                    x: x / norm,
                    y: y / norm,
                    z: z / norm,
                    w: w / norm,
                };

                let distance = self.space.distance(&self.target, &random_quat);
                if distance <= self.radius {
                    return Ok(random_quat);
                }
            }
        }
    }
}

/// A helper function to validate the entire solution path.
fn is_path_valid(
    path: &Path<SO3State>,
    space: &SO3StateSpace,
    checker: &dyn StateValidityChecker<SO3State>,
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
        let num_steps = (dist / space.get_longest_valid_segment_length()).ceil() as usize;
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
fn test_rrt_connect_finds_path_in_so3ss() {
    let space = Arc::new(SO3StateSpace::new(None).unwrap());

    let start_state = quaternion_from_axis_angle([0.0, 1.0, 0.0], PI / 2.0);

    let goal_target = quaternion_from_axis_angle([0.0, 1.0, 0.0], -PI / 2.0);

    let goal_definition = Arc::new(SO3GoalRegion {
        target: goal_target,
        radius: 10.0f64.to_radians(),
        space: space.clone(),
    });

    let problem_definition = Arc::new(ProblemDefinition {
        space: space.clone(),
        start_states: vec![start_state.clone()],
        goal: goal_definition.clone(),
    });

    let validity_checker = Arc::new(ForbiddenConeChecker {
        center: SO3State::identity(),
        radius: 40.0f64.to_radians(),
        space: space.clone(),
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

    let mut planner = RRTConnect::new(0.2, 0.05);

    planner.setup(problem_definition, validity_checker.clone());

    let timeout = Duration::from_secs(10);
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

    println!("RRT planner test passed!");
}
