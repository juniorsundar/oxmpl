// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::{
    error::PlanningError, goal::Goal, problem_definition::ProblemDefinition, space::StateSpace,
    state::State, validity::StateValidityChecker,
};
use std::{sync::Arc, time::Duration};

/// Represents a solution path found by a planner.
///
/// "newtype" wrapper around a `Vec<S>` to provide stronger type safety. It makes it clear that
/// this vector of states represents a complete, ordered path from a start state to a goal
/// state.
#[derive(Clone)]
pub struct Path<S: State>(pub Vec<S>);

/// The central trait for all motion planning algorithms.
///
/// A `Planner` is responsible for finding a valid `Path` that connects a start state to a goal,
/// while respecting the constraints defined by a `StateSpace` and a `StateValidityChecker`.
///
/// This trait is highly generic, allowing specific planner implementations (like RRT) to be
/// written once and then used to solve problems in many different state spaces.
pub trait Planner<S: State, SP: StateSpace<StateType = S>, G: Goal<S>> {
    /// Configures the planner for a specific motion planning problem.
    ///
    /// This method **MUST** be called before `solve()`. It provides the planner with all the
    /// necessary context, including the state space, start/goal definitions, and the validity
    /// checker for collision detection.
    ///
    /// The planner is expected to store this information and perform any necessary initialization
    /// of its internal data structures.
    ///
    /// # Parameters
    ///
    /// * `problem_def` - An `Arc`-wrapped `ProblemDefinition` containing the state space, start
    ///   states, and goal condition.
    /// * `validity_checker` - An `Arc`-wrapped `StateValidityChecker` that the planner will use to
    ///   determine if states or motions are valid.
    fn setup(
        &mut self,
        problem_def: Arc<ProblemDefinition<S, SP, G>>,
        validity_checker: Arc<dyn StateValidityChecker<S>>,
    );

    /// Attempt to find a solution to the planning problem.
    ///
    /// # Parameters
    ///
    /// * `timeout` - The maximum `Duration` the planner is allowed to run before giving up.
    ///
    /// # Returns
    ///
    /// * `Ok(Path<S>)` if a solution is found. The `Path` contains the sequence of states.
    /// * `Err(PlanningError)` if no solution is found within the timeout, or if another error
    ///   occurs.
    fn solve(&mut self, timeout: Duration) -> Result<Path<S>, PlanningError>;
}

/// Configuration for a planner.
#[derive(Default)]
pub struct PlannerConfig {
    /// The seed for the random number generator.
    pub seed: Option<u64>,
}
