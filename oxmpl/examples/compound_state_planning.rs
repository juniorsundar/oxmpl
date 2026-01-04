use std::{f64::consts::PI, sync::Arc, time::Duration};

use oxmpl::base::{
    error::StateSamplingError,
    goal::{Goal, GoalRegion, GoalSampleableRegion},
    planner::{Planner, PlannerConfig},
    problem_definition::ProblemDefinition,
    space::{CompoundStateSpace, RealVectorStateSpace, SO2StateSpace, StateSpace},
    state::{CompoundState, RealVectorState, SO2State},
    validity::StateValidityChecker,
};
use oxmpl::geometric::RRT;

use rand::Rng;

/// A StateValidityChecker that checks for collision with a box obstacle.
/// The robot is considered a point with orientation, but we check if the point
/// is inside the box.
struct BoxObstacleChecker {
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
}

impl StateValidityChecker<CompoundState> for BoxObstacleChecker {
    fn is_valid(&self, state: &CompoundState) -> bool {
        // Extract the RealVector component (index 0)
        if let Some(rv_state) = state.components[0]
            .as_any()
            .downcast_ref::<RealVectorState>()
        {
            let x = rv_state.values[0];
            let y = rv_state.values[1];

            // Check if inside the box
            let inside_box =
                x >= self.x_min && x <= self.x_max && y >= self.y_min && y <= self.y_max;

            !inside_box
        } else {
            // Should not happen if constructed correctly
            false
        }
    }
}

/// A Goal definition for a CompoundState (Position + Orientation).
/// Success is being within a radius of the target position.
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
        // Get target components
        let target_rv = self.target.components[0]
            .as_any()
            .downcast_ref::<RealVectorState>()
            .unwrap();

        // Sample a position around the target within radius
        let angle = rng.random_range(0.0..2.0 * PI);
        let radius = self.radius * rng.random::<f64>().sqrt();

        let x = target_rv.values[0] + radius * angle.cos();
        let y = target_rv.values[1] + radius * angle.sin();

        // Sample a random orientation for the goal (or keep it fixed if desired)
        // Here we sample a random orientation effectively ignoring orientation for the goal region
        let theta = rng.random_range(0.0..2.0 * PI);

        Ok(CompoundState {
            components: vec![
                Box::new(RealVectorState { values: vec![x, y] }),
                Box::new(SO2State::new(theta)),
            ],
        })
    }
}

fn main() {
    // 1. Define the state space: R^2 x SO(2)
    // R^2 for position (x, y)
    let r2 = RealVectorStateSpace::new(2, Some(vec![(-5.0, 5.0), (-5.0, 5.0)]))
        .expect("Failed to create R2 space");

    // SO(2) for orientation (theta)
    let so2 = SO2StateSpace::new(None).expect("Failed to create SO2 space");

    // Combine them into a CompoundStateSpace
    // Weights: 1.0 for position, 0.5 for orientation (orientation matters less for distance)
    let space = Arc::new(CompoundStateSpace::new(
        vec![Box::new(r2), Box::new(so2)],
        vec![1.0, 0.5],
    ));

    // 2. Define Start and Goal states
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

    // 3. Define Goal Region
    let goal_definition = Arc::new(CompoundGoalRegion {
        target: target_state.clone(),
        radius: 0.5,
        space: space.clone(),
    });

    // 4. Define Problem
    let problem_definition = Arc::new(ProblemDefinition {
        space: space.clone(),
        start_states: vec![start_state.clone()],
        goal: goal_definition.clone(),
    });

    // 5. Define Validity Checker (Box Obstacle)
    let validity_checker = Arc::new(BoxObstacleChecker {
        x_min: -0.5,
        x_max: 0.5,
        y_min: -2.0,
        y_max: 2.0,
    });

    // 6. Setup Planner (RRT)
    let mut planner = RRT::new(0.5, 0.1, &PlannerConfig { seed: Some(123) });
    planner.setup(problem_definition, validity_checker.clone());

    // 7. Solve
    println!("Solving Compound State planning problem...");
    let timeout = Duration::from_secs(5);
    match planner.solve(timeout) {
        Ok(path) => {
            println!("Solution found with {} states.", path.0.len());

            // Print first and last state to verify
            if let Some(first) = path.0.first() {
                let rv = first.components[0]
                    .as_any()
                    .downcast_ref::<RealVectorState>()
                    .unwrap();
                let so2 = first.components[1]
                    .as_any()
                    .downcast_ref::<SO2State>()
                    .unwrap();
                println!(
                    "Start: ({:.2}, {:.2}, {:.2})",
                    rv.values[0], rv.values[1], so2.value
                );
            }
            if let Some(last) = path.0.last() {
                let rv = last.components[0]
                    .as_any()
                    .downcast_ref::<RealVectorState>()
                    .unwrap();
                let so2 = last.components[1]
                    .as_any()
                    .downcast_ref::<SO2State>()
                    .unwrap();
                println!(
                    "End:   ({:.2}, {:.2}, {:.2})",
                    rv.values[0], rv.values[1], so2.value
                );
            }
        }
        Err(e) => println!("Planner failed: {:?}", e),
    }
}
