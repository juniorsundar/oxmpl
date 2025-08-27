use std::{f64::consts::PI, sync::Arc, time::Duration};

use oxmpl::base::{
    error::StateSamplingError,
    goal::{Goal, GoalRegion, GoalSampleableRegion},
    planner::{Path, Planner},
    problem_definition::ProblemDefinition,
    space::{SO2StateSpace, StateSpace},
    state::SO2State,
    validity::StateValidityChecker,
};
use oxmpl::geometric::PRM;

use rand::Rng;

/// A StateValidityChecker that defines a forbidden range of angles.
struct ForbiddenAngleChecker {
    invalid_min: f64,
    invalid_max: f64,
}

impl StateValidityChecker<SO2State> for ForbiddenAngleChecker {
    fn is_valid(&self, state: &SO2State) -> bool {
        let val = state.clone().normalise().value;
        !(val >= self.invalid_min && val <= self.invalid_max)
    }
}

struct AngleGoalRegion {
    target: SO2State,
    radius: f64,
    space: Arc<SO2StateSpace>,
}

impl Goal<SO2State> for AngleGoalRegion {
    fn is_satisfied(&self, state: &SO2State) -> bool {
        self.space.distance(state, &self.target) <= self.radius
    }
}

impl GoalRegion<SO2State> for AngleGoalRegion {
    fn distance_goal(&self, state: &SO2State) -> f64 {
        let dist_to_center = self.space.distance(state, &self.target);
        (dist_to_center - self.radius).max(0.0)
    }
}

impl GoalSampleableRegion<SO2State> for AngleGoalRegion {
    fn sample_goal(&self, rng: &mut impl Rng) -> Result<SO2State, StateSamplingError> {
        let random_offset = rng.random_range(-self.radius..=self.radius);
        Ok(SO2State::new(self.target.value + random_offset))
    }
}

/// A helper function to validate the entire solution path in SO(2).
fn is_so2_path_valid(
    path: &Path<SO2State>,
    space: &SO2StateSpace,
    checker: &dyn StateValidityChecker<SO2State>,
) -> bool {
    for i in 0..path.0.len() - 1 {
        let state_a = &path.0[i];
        let state_b = &path.0[i + 1];

        if !checker.is_valid(state_a) {
            println!("Path invalid: State {:?} is in collision.", state_a);
            return false;
        }
        if (i + 1 == path.0.len() - 1) && !checker.is_valid(state_b) {
            println!("Path invalid: Final state {:?} is in collision.", state_b);
            return false;
        }

        // Use the space's own resolution hint for checking motions
        let dist = space.distance(state_a, state_b);
        let num_steps = (dist / space.get_longest_valid_segment_length()).ceil() as usize;
        if num_steps > 1 {
            let mut interpolated_state = state_a.clone();
            for j in 1..=num_steps {
                let t = j as f64 / num_steps as f64;
                space.interpolate(state_a, state_b, t, &mut interpolated_state);
                if !checker.is_valid(&interpolated_state) {
                    println!(
                        "Path invalid: Motion between {:?} and {:?} is in collision at {:?}.",
                        state_a, state_b, interpolated_state
                    );
                    return false;
                }
            }
        }
    }
    true
}

#[test]
fn test_rrt_finds_path_in_so2ss() {
    let space = Arc::new(SO2StateSpace::new(None).expect("Failed to create state space"));

    let start_state = SO2State::new(-PI / 2.0);
    let goal_definition = Arc::new(AngleGoalRegion {
        target: SO2State::new(PI / 2.0),
        radius: 0.1,
        space: space.clone(),
    });

    let problem_definition = Arc::new(ProblemDefinition {
        space: space.clone(),
        start_states: vec![start_state.clone()],
        goal: goal_definition.clone(),
    });

    let validity_checker = Arc::new(ForbiddenAngleChecker {
        invalid_min: -0.5,
        invalid_max: 0.5,
    });
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
        is_so2_path_valid(&path, &space, &*validity_checker),
        "The returned path was found to be invalid."
    );

    println!("PRM planner test passed!");
}
