use std::{f64::consts::PI, sync::Arc, time::Duration};

use oxmpl::base::{
    error::StateSamplingError,
    goal::{Goal, GoalRegion, GoalSampleableRegion},
    planner::{Path, Planner, PlannerConfig},
    problem_definition::ProblemDefinition,
    space::{CompoundStateSpace, RealVectorStateSpace, SO2StateSpace, StateSpace},
    state::{CompoundState, RealVectorState, SO2State},
    validity::StateValidityChecker,
};
use oxmpl::geometric::PRM;

use rand::Rng;

// A compound space of R2 + SO2 (similar to SE2 but explicit compound)
struct BoxObstacleChecker {
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
}

impl StateValidityChecker<CompoundState> for BoxObstacleChecker {
    fn is_valid(&self, state: &CompoundState) -> bool {
        if let Some(rv_state) = state.components[0]
            .as_any()
            .downcast_ref::<RealVectorState>()
        {
            let x = rv_state.values[0];
            let y = rv_state.values[1];
            !(x >= self.x_min && x <= self.x_max && y >= self.y_min && y <= self.y_max)
        } else {
            false
        }
    }
}

struct CompoundGoalRegion {
    target: CompoundState,
    radius: f64,
    space: Arc<CompoundStateSpace>,
}

impl Goal<CompoundState> for CompoundGoalRegion {
    fn is_satisfied(&self, state: &CompoundState) -> bool {
        self.space.distance(state, &self.target) <= self.radius
    }
}

impl GoalRegion<CompoundState> for CompoundGoalRegion {
    fn distance_goal(&self, state: &CompoundState) -> f64 {
        let dist = self.space.distance(state, &self.target);
        (dist - self.radius).max(0.0)
    }
}

impl GoalSampleableRegion<CompoundState> for CompoundGoalRegion {
    fn sample_goal(&self, rng: &mut impl Rng) -> Result<CompoundState, StateSamplingError> {
        let target_rv = self.target.components[0]
            .as_any()
            .downcast_ref::<RealVectorState>()
            .unwrap();
        let angle = rng.random_range(0.0..2.0 * PI);
        let r = rng.random_range(0.0..self.radius);
        let x = target_rv.values[0] + r * angle.cos();
        let y = target_rv.values[1] + r * angle.sin();
        let rv_sample = RealVectorState::new(vec![x, y]);

        let so2_sample = SO2State::new(rng.random_range(-PI..PI));

        Ok(CompoundState {
            components: vec![Box::new(rv_sample), Box::new(so2_sample)],
        })
    }
}

fn is_path_valid(
    path: &Path<CompoundState>,
    space: &CompoundStateSpace,
    checker: &dyn StateValidityChecker<CompoundState>,
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
    if let Some(last) = path.0.last() {
        if !checker.is_valid(last) {
            return false;
        }
    }
    true
}

#[test]
#[allow(clippy::arc_with_non_send_sync)]
fn test_prm_finds_path_in_css() {
    let r2 = RealVectorStateSpace::new(2, Some(vec![(-5.0, 5.0), (-5.0, 5.0)])).unwrap();
    let so2 = SO2StateSpace::new(None).unwrap();
    let space = Arc::new(CompoundStateSpace::new(
        vec![Box::new(r2), Box::new(so2)],
        vec![1.0, 0.5],
    ));

    let start_state = CompoundState {
        components: vec![
            Box::new(RealVectorState::new(vec![-2.0, 0.0])),
            Box::new(SO2State::new(0.0)),
        ],
    };

    let target_state = CompoundState {
        components: vec![
            Box::new(RealVectorState::new(vec![2.0, 0.0])),
            Box::new(SO2State::new(PI)),
        ],
    };

    let goal_definition = Arc::new(CompoundGoalRegion {
        target: target_state.clone(),
        radius: 0.5,
        space: space.clone(),
    });

    let problem_definition = Arc::new(ProblemDefinition {
        space: space.clone(),
        start_states: vec![start_state.clone()],
        goal: goal_definition.clone(),
    });

    let validity_checker = Arc::new(BoxObstacleChecker {
        x_min: -0.5,
        x_max: 0.5,
        y_min: -2.0,
        y_max: 2.0,
    });

    assert!(validity_checker.is_valid(&start_state));
    assert!(validity_checker.is_valid(&target_state));

    let mut planner = PRM::new(10.0, 2.0, &PlannerConfig { seed: Some(0) });
    planner.setup(problem_definition, validity_checker.clone());
    planner.construct_roadmap().unwrap();

    let result = planner.solve(Duration::from_secs(5));
    assert!(result.is_ok());

    let path = result.unwrap();
    assert!(!path.0.is_empty());
    assert!(space.distance(path.0.first().unwrap(), &start_state) < 1e-9);
    assert!(goal_definition.is_satisfied(path.0.last().unwrap()));
    assert!(is_path_valid(&path, &space, &*validity_checker));

    println!("PRM CSS test passed!");
}
