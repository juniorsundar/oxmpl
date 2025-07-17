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

pub struct RRTConnect<S: State, SP: StateSpace<StateType = S>, G: Goal<S>> {
    pub max_distance: f64,
    pub goal_bias: f64,

    problem_def: Option<Arc<ProblemDefinition<S, SP, G>>>,
    validity_checker: Option<Arc<dyn StateValidityChecker<S>>>,
    tree_a: Vec<Node<S>>,
    tree_b: Vec<Node<S>>,
}

impl<S, SP, G> RRTConnect<S, SP, G>
where
    S: State,
    SP: StateSpace<StateType = S>,
    G: Goal<S> + GoalSampleableRegion<S>,
{
    pub fn new(max_distance: f64, goal_bias: f64) -> Self {
        RRTConnect {
            max_distance,
            goal_bias,
            problem_def: None,
            validity_checker: None,
            tree_a: Vec::new(),
            tree_b: Vec::new(),
        }
    }

    fn extend_branch(
        &self,
        tree: &[Node<S>],
        q_target: &S,
        max_distance: &f64,
        problem_def: &ProblemDefinition<S, SP, G>,
    ) -> Result<(S, usize), PlanningError> {
        let mut nearest_node_index = 0;
        let mut min_dist = problem_def.space.distance(&tree[0].state, q_target);

        for (i, node) in tree.iter().enumerate().skip(1) {
            let dist = problem_def.space.distance(&node.state, q_target);
            if dist < min_dist {
                min_dist = dist;
                nearest_node_index = i;
            }
        }

        let q_near = &tree[nearest_node_index].state;

        let mut q_new = q_near.clone();
        if min_dist > *max_distance {
            let t = max_distance / min_dist;
            problem_def
                .space
                .interpolate(q_near, q_target, t, &mut q_new);
        } else {
            q_new = q_target.clone();
        }

        if self.check_motion(q_near, &q_new) {
            Ok((q_new, nearest_node_index))
        } else {
            Err(PlanningError::PlannerUninitialised)
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

    fn reconstruct_path(&self, start_node_idx: usize, goal_node_idx: usize) -> Path<S> {
        let mut path_states = Vec::new();

        let mut current_index = Some(start_node_idx);
        while let Some(index) = current_index {
            path_states.push(self.tree_a[index].state.clone());
            current_index = self.tree_a[index].parent_index;
        }
        path_states.reverse();

        let mut current_index = Some(goal_node_idx);
        while let Some(index) = current_index {
            if index != goal_node_idx {
                path_states.push(self.tree_b[index].state.clone());
            }
            current_index = self.tree_b[index].parent_index;
        }

        Path(path_states)
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
        self.tree_a.clear();
        self.tree_b.clear();
        let pd = self.problem_def.as_ref().unwrap();

        let start_state = pd.start_states[0].clone();
        let start_node = Node {
            state: start_state,
            parent_index: None,
        };
        self.tree_a.push(start_node);

        let mut rng = rand::rng();
        let goal_state = pd.goal.sample_goal(&mut rng).unwrap();
        let goal_node = Node {
            state: goal_state,
            parent_index: None,
        };
        self.tree_b.push(goal_node);
    }

    fn solve(&mut self, timeout: Duration) -> Result<Path<S>, PlanningError> {
        let start_time = Instant::now();
        let mut rng = rand::rng();
        let pd = self
            .problem_def
            .as_ref()
            .ok_or(PlanningError::PlannerUninitialised)?;

        loop {
            if start_time.elapsed() > timeout {
                return Err(PlanningError::Timeout);
            }

            let goal = &pd.goal;

            let q_rand = if rng.random_bool(self.goal_bias) {
                goal.sample_goal(&mut rng).unwrap()
            } else {
                pd.space.sample_uniform(&mut rng).unwrap()
            };

            if let Ok((q_new_from_start, nn_idx_new_from_start)) =
                self.extend_branch(&self.tree_a, &q_rand, &self.max_distance, pd)
            {
                self.tree_a.push(Node {
                    state: q_new_from_start.clone(),
                    parent_index: Some(nn_idx_new_from_start),
                });
                if let Ok((q_connect_from_goal, nn_idx_connect_from_goal)) =
                    self.extend_branch(&self.tree_b, &q_new_from_start, &self.max_distance, pd)
                {
                    self.tree_a.push(Node {
                        state: q_connect_from_goal.clone(),
                        parent_index: Some(nn_idx_connect_from_goal),
                    });

                    if pd.space.distance(&q_new_from_start, &q_connect_from_goal) < 1e-9 {
                        println!(
                            "Solution found after {} total nodes.",
                            self.tree_a.len() + self.tree_b.len()
                        );
                        return Ok(
                            self.reconstruct_path(self.tree_a.len() - 1, self.tree_b.len() - 1)
                        );
                    }
                }
            }

            mem::swap(&mut self.tree_a, &mut self.tree_b);
        }
    }
}
