// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use std::{
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

pub struct RRTConnect<S: State, SP: StateSpace<StateType = S>, G: Goal<S>> {
    pub max_distance: f64,
    pub goal_bias: f64,

    problem_def: Option<Arc<ProblemDefinition<S, SP, G>>>,
    validity_checker: Option<Arc<dyn StateValidityChecker<S>>>,
    tree: Vec<Node<S>>,
}

impl<S, SP, G> RRTConnect<S, SP, G>
where
    S: State,
    SP: StateSpace<StateType = S>,
    G: Goal<S>,
{
    pub fn new(max_distance: f64, goal_bias: f64) -> Self {
        RRTConnect {
            max_distance,
            goal_bias,
            problem_def: None,
            validity_checker: None,
            tree: Vec::new(),
        }
    }

    fn check_motion(&self, from: &S, to: &S) -> bool {
        if let (Some(pd), Some(vc)) = (&self.problem_def, &self.validity_checker) {
            let space = &pd.space;

            let dist = space.distance(from, to);
            let num_steps = (dist / (self.max_distance * 0.1)).ceil() as usize;

            if num_steps <= 1 {
                return vc.is_valid(to);
            }

            let mut interpolated_state = from.clone();
            for i in 1..=num_steps {
                let t = i as f64 / num_steps as f64;
                space.interpolate(from, to, t, &mut interpolated_state);
                if !vc.is_valid(&interpolated_state) {
                    return false;
                }
            }
            true
        } else {
            false
        }
    }
}

impl<S, SP, G> Planner<S, SP, G> for RRTConnect<S, SP, G>
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
        self.problem_def = Some(problem_def);
        self.validity_checker = Some(validity_checker);
        self.tree.clear();

        let start_state = self.problem_def.as_ref().unwrap().start_states[0].clone();
        let start_node = Node {
            state: start_state,
            parent_index: None,
        };
        self.tree.push(start_node);
    }

    fn solve(&mut self, timeout: Duration) -> Result<Path<S>, PlanningError> {
        let pd = self
            .problem_def
            .as_ref()
            .expect("Planner setup was not called before solve.");
        let goal = &pd.goal;

        let start_time = Instant::now();
        let mut rng = rand::rng();

        loop {
            if start_time.elapsed() > timeout {
                return Err(PlanningError::Timeout);
            }

            let q_rand = if rng.random_bool(self.goal_bias) {
                goal.sample_goal(&mut rng).unwrap()
            } else {
                pd.space.sample_uniform(&mut rng).unwrap()
            };

            let mut nearest_node_index = 0;
            let mut min_dist = pd.space.distance(&self.tree[0].state, &q_rand);

            for i in 1..self.tree.len() {
                let dist = pd.space.distance(&self.tree[i].state, &q_rand);
                if dist < min_dist {
                    min_dist = dist;
                    nearest_node_index = i;
                }
            }
            let q_near = &self.tree[nearest_node_index].state;

            let mut q_new = q_near.clone();
            if min_dist > self.max_distance {
                let t = self.max_distance / min_dist;
                pd.space.interpolate(q_near, &q_rand, t, &mut q_new);
            } else {
                q_new = q_rand;
            }

            if self.check_motion(q_near, &q_new) {
                let new_node = Node {
                    state: q_new.clone(),
                    parent_index: Some(nearest_node_index),
                };
                let new_node_index = self.tree.len();
                self.tree.push(new_node);

                if goal.is_satisfied(&q_new) {
                    println!("Solution found after {} nodes.", self.tree.len());
                    let mut path_states = Vec::new();
                    let mut current_index = Some(new_node_index);
                    while let Some(index) = current_index {
                        path_states.push(self.tree[index].state.clone());
                        current_index = self.tree[index].parent_index;
                    }
                    path_states.reverse();
                    return Ok(Path(path_states));
                }
            }
        }
    }
}
