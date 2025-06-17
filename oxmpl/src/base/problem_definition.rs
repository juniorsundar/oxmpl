// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::{goal::Goal, space::StateSpace, state::State};
use std::sync::Arc;

/// Encapsulates the definition of a complete motion planning problem.
///
/// This struct brings together all the necessary components that define a planning problem: the
/// space in which to plan, one or more start states, and the goal condition. It is passed to a
/// `Planner`'s `setup` method to configure it for a specific task.
///
/// The fields use `Arc` to allow for safe, shared ownership of the space and goal definitions,
/// which might be referenced by multiple parts of the planning process.
pub struct ProblemDefinition<S: State, SP: StateSpace<StateType = S>, G: Goal<S>> {
    pub space: Arc<SP>,
    pub start_states: Vec<S>,
    pub goal: Arc<G>,
}
