// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use std::{
    mem,
    sync::Arc,
    time::{Duration, Instant},
};

use rand::Rng;

use crate::base::{
    error::PlanningError,
    goal::{Goal, GoalSampleableRegion},
    planner::{Path, Planner},
    problem_definition::ProblemDefinition,
    space::StateSpace,
    state::State,
    validity::StateValidityChecker,
};

#[derive(Clone)]
struct Node<S: State> {
    state: S,
    parent_index: Option<usize>,
}

pub struct RRTStar<S: State, SP: StateSpace<StateType = S>, G: Goal<S>> {}

impl<S, SP, G> RRTStar<S, SP, G>
where
    S: State,
    SP: StateSpace<StateType = S>,
    G: Goal<S> + GoalSampleableRegion<S>,
{
}

impl<S, SP, G> Planner<S, SP, G> for RRTStar<S, SP, G>
where
    S: State + Clone,
    SP: StateSpace<StateType = S>,
    G: Goal<S> + GoalSampleableRegion<S>,
{
    fn setup(
        &mut self,
        problem_def: Arc<ProblemDefinition<S, SP, G>>,
        validity_checker: Arc<dyn StateValidityChecker<S>>,
    ) {
    }

    fn solve(&mut self, timeout: Duration) -> Result<Path<S>, PlanningError> {}
}

