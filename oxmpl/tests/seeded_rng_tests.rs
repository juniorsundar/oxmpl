use std::{
    cell::{Cell, RefCell},
    rc::Rc,
    sync::Arc,
    time::Duration,
};

use oxmpl::{
    base::{
        error::{PlanningError, StateSamplingError},
        goal::{Goal, GoalRegion, GoalSampleableRegion},
        planner::{Planner, PlannerConfig},
        problem_definition::ProblemDefinition,
        space::{RealVectorStateSpace, StateSpace},
        state::RealVectorState,
        validity::StateValidityChecker,
    },
    geometric::{PRM, RRT, RRTConnect, RRTStar},
};
use rand::{Rng, RngExt, SeedableRng, rngs::StdRng};

const SEED: u64 = 42;
const SOLVE_TIMEOUT: Duration = Duration::from_secs(30);

struct Samples {
    reference: RefCell<(StdRng, usize)>,
    fail: Cell<bool>,
}

impl Samples {
    fn new() -> Self {
        Self {
            reference: RefCell::new((StdRng::seed_from_u64(SEED), 0)),
            fail: Cell::new(false),
        }
    }

    fn count(&self) -> usize {
        self.reference.borrow().1
    }

    fn sample(&self, rng: &mut impl Rng) -> Result<RealVectorState, StateSamplingError> {
        let draw = rng.random::<f64>();
        let mut reference = self.reference.borrow_mut();
        assert_eq!(
            draw,
            reference.0.random::<f64>(),
            "planner must continue the seeded stream at draw {}",
            reference.1,
        );
        reference.1 += 1;

        if self.fail.get() {
            return Err(StateSamplingError::ZeroVolume);
        }
        Ok(RealVectorState { values: vec![draw] })
    }
}

struct SamplingSpace {
    inner: RealVectorStateSpace,
    samples: Rc<Samples>,
}

impl StateSpace for SamplingSpace {
    type StateType = RealVectorState;

    fn distance(&self, from: &RealVectorState, to: &RealVectorState) -> f64 {
        self.inner.distance(from, to)
    }

    fn interpolate(
        &self,
        from: &RealVectorState,
        to: &RealVectorState,
        t: f64,
        state: &mut RealVectorState,
    ) {
        self.inner.interpolate(from, to, t, state);
    }

    fn enforce_bounds(&self, state: &mut RealVectorState) {
        self.inner.enforce_bounds(state);
    }

    fn satisfies_bounds(&self, state: &RealVectorState) -> bool {
        self.inner.satisfies_bounds(state)
    }

    fn sample_uniform(&self, rng: &mut impl Rng) -> Result<RealVectorState, StateSamplingError> {
        self.samples.sample(rng)
    }

    fn get_longest_valid_segment_length(&self) -> f64 {
        self.inner.get_longest_valid_segment_length()
    }
}

// The entire bounded space is a goal region, so a valid extension solves the
// problem immediately.
struct SamplingGoal(Rc<Samples>);

impl Goal<RealVectorState> for SamplingGoal {
    fn is_satisfied(&self, _state: &RealVectorState) -> bool {
        true
    }
}

impl GoalRegion<RealVectorState> for SamplingGoal {
    fn distance_goal(&self, _state: &RealVectorState) -> f64 {
        0.0
    }
}

impl GoalSampleableRegion<RealVectorState> for SamplingGoal {
    fn sample_goal(&self, rng: &mut impl Rng) -> Result<RealVectorState, StateSamplingError> {
        self.0.sample(rng)
    }
}

struct Validity(bool);

impl StateValidityChecker<RealVectorState> for Validity {
    fn is_valid(&self, _state: &RealVectorState) -> bool {
        self.0
    }
}

// The public planner API requires Arc; this fixture is intentionally single-threaded.
#[allow(clippy::arc_with_non_send_sync)]
fn problem(
    samples: Rc<Samples>,
) -> Arc<ProblemDefinition<RealVectorState, SamplingSpace, SamplingGoal>> {
    Arc::new(ProblemDefinition {
        space: Arc::new(SamplingSpace {
            inner: RealVectorStateSpace::new(1, Some(vec![(0.0, 1.0)])).unwrap(),
            samples: samples.clone(),
        }),
        start_states: vec![RealVectorState { values: vec![0.0] }],
        goal: Arc::new(SamplingGoal(samples)),
    })
}

fn assert_tree_stream_continues(
    mut planner: impl Planner<RealVectorState, SamplingSpace, SamplingGoal>,
) {
    let samples = Rc::new(Samples::new());
    planner.setup(problem(samples.clone()), Arc::new(Validity(true)));

    assert!(planner.solve(SOLVE_TIMEOUT).is_ok());
    let after_success = samples.count();
    assert!(after_success > 0);

    assert!(matches!(
        planner.solve(Duration::ZERO),
        Err(PlanningError::Timeout)
    ));
    assert_eq!(samples.count(), after_success);

    samples.fail.set(true);
    assert!(matches!(
        planner.solve(SOLVE_TIMEOUT),
        Err(PlanningError::Sampling(StateSamplingError::ZeroVolume))
    ));
    assert_eq!(samples.count(), after_success + 1);

    samples.fail.set(false);
    assert!(planner.solve(SOLVE_TIMEOUT).is_ok());
    assert!(samples.count() > after_success + 1);
}

#[test]
fn rrt_keeps_seeded_stream_after_success_timeout_and_sampling_error() {
    // Bias 1.0 always samples the goal and consumes no random branch-selection
    // draw.
    assert_tree_stream_continues(RRT::new(2.0, 1.0, &PlannerConfig { seed: Some(SEED) }));
}

#[test]
fn rrt_star_keeps_seeded_stream_after_success_timeout_and_sampling_error() {
    assert_tree_stream_continues(RRTStar::new(
        2.0,
        1.0,
        2.0,
        &PlannerConfig { seed: Some(SEED) },
    ));
}

#[test]
fn rrt_connect_keeps_seeded_stream_after_success_timeout_and_sampling_error() {
    assert_tree_stream_continues(RRTConnect::new(
        2.0,
        1.0,
        &PlannerConfig { seed: Some(SEED) },
    ));
}

#[test]
fn rrt_connect_keeps_seeded_stream_after_goal_seed_error_and_timeout() {
    let samples = Rc::new(Samples::new());
    let mut planner = RRTConnect::new(2.0, 1.0, &PlannerConfig { seed: Some(SEED) });
    planner.setup(problem(samples.clone()), Arc::new(Validity(true)));

    samples.fail.set(true);
    assert!(matches!(
        planner.solve(SOLVE_TIMEOUT),
        Err(PlanningError::Sampling(StateSamplingError::ZeroVolume))
    ));
    assert_eq!(samples.count(), 1);

    samples.fail.set(false);
    // RRTConnect seeds its goal tree before checking the solve timeout.
    assert!(matches!(
        planner.solve(Duration::ZERO),
        Err(PlanningError::Timeout)
    ));
    assert_eq!(samples.count(), 2);

    assert!(planner.solve(SOLVE_TIMEOUT).is_ok());
    assert_eq!(samples.count(), 3);
}

#[test]
fn prm_keeps_seeded_stream_after_completed_construction_and_sampling_errors() {
    let samples = Rc::new(Samples::new());
    let mut planner = PRM::new(1.0, 0.5, &PlannerConfig { seed: Some(SEED) });
    // Reject milestones so a later construction actually samples again instead
    // of returning early for a nonempty roadmap.
    planner.setup(problem(samples.clone()), Arc::new(Validity(false)));

    assert!(planner.construct_roadmap().is_ok());
    let after_success = samples.count();
    assert!(after_success > 0);
    assert!(planner.get_roadmap().is_empty());

    samples.fail.set(true);
    for retry in 1..=2 {
        assert!(matches!(
            planner.construct_roadmap(),
            Err(PlanningError::Sampling(StateSamplingError::ZeroVolume))
        ));
        assert_eq!(samples.count(), after_success + retry);
    }

    samples.fail.set(false);
    assert!(planner.construct_roadmap().is_ok());
    assert!(samples.count() > after_success + 2);
}
