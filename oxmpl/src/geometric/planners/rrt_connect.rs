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

#[derive(PartialEq, Debug)]
enum ExtendResult {
    /// The tree was extended, but did not reach the target state.
    Advanced,
    /// The tree was extended and reached the target state exactly.
    Reached,
}

pub struct RRTConnect<S: State, SP: StateSpace<StateType = S>, G: Goal<S>> {
    pub max_distance: f64,
    pub goal_bias: f64,

    problem_def: Option<Arc<ProblemDefinition<S, SP, G>>>,
    validity_checker: Option<Arc<dyn StateValidityChecker<S>>>,
    start_tree: Vec<Node<S>>,
    goal_tree: Vec<Node<S>>,
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
            start_tree: Vec::new(),
            goal_tree: Vec::new(),
        }
    }

    fn reconstruct_path(&self, tree: &[Node<S>], last_node_idx: usize) -> Path<S> {
        let mut path_states = Vec::new();
        let mut current_index = Some(last_node_idx);
        while let Some(index) = current_index {
            path_states.push(tree[index].state.clone());
            current_index = tree[index].parent_index;
        }
        path_states.reverse();
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
        self.start_tree.clear();
        self.goal_tree.clear();
        let pd = self.problem_def.as_ref().unwrap();

        let start_state = pd.start_states[0].clone();
        let start_node = Node {
            state: start_state,
            parent_index: None,
        };
        self.start_tree.push(start_node);

        let mut rng = rand::rng();
        let goal_state = pd.goal.sample_goal(&mut rng).unwrap();
        let goal_node = Node {
            state: goal_state,
            parent_index: None,
        };
        self.goal_tree.push(goal_node);
    }

    fn solve(&mut self, timeout: Duration) -> Result<Path<S>, PlanningError> {
        let start_time = Instant::now();
        let mut rng = rand::rng();
        let pd = self
            .problem_def
            .as_ref()
            .ok_or(PlanningError::PlannerUninitialised)?;
        let vc = self
            .validity_checker
            .as_ref()
            .ok_or(PlanningError::PlannerUninitialised)?;
        let goal = &pd.goal;

        loop {
            if start_time.elapsed() > timeout {
                return Err(PlanningError::Timeout);
            }

            let (tree_a, tree_b, is_growing_start_tree) =
                if self.start_tree.len() <= self.goal_tree.len() {
                    (&mut self.start_tree, &mut self.goal_tree, true)
                } else {
                    (&mut self.goal_tree, &mut self.start_tree, false)
                };

            let q_rand = if rng.random_bool(self.goal_bias) {
                goal.sample_goal(&mut rng).unwrap()
            } else {
                pd.space.sample_uniform(&mut rng).unwrap()
            };

            if let Some((_extend_result, new_node_idx_a)) =
                extend(tree_a, &q_rand, pd, vc, self.max_distance)
            {
                let q_new = &tree_a[new_node_idx_a].state;

                if is_growing_start_tree && goal.is_satisfied(q_new) {
                    println!("Solution found by start tree reaching goal directly.");
                    return Ok(self.reconstruct_path(&self.start_tree, new_node_idx_a));
                }

                if let Some((connect_result, new_node_idx_b)) =
                    extend(tree_b, q_new, pd, vc, self.max_distance)
                {
                    if connect_result == ExtendResult::Reached {
                        println!(
                            "Solution found after {} total nodes.",
                            self.start_tree.len() + self.goal_tree.len()
                        );

                        let (start_idx, goal_idx) = if is_growing_start_tree {
                            (new_node_idx_a, new_node_idx_b)
                        } else {
                            (new_node_idx_b, new_node_idx_a)
                        };

                        let mut start_path = self.reconstruct_path(&self.start_tree, start_idx).0;
                        let mut goal_path = self.reconstruct_path(&self.goal_tree, goal_idx).0;

                        goal_path.reverse();
                        start_path.extend(goal_path.into_iter().skip(1));

                        return Ok(Path(start_path));
                    }
                }
            }
        }
    }
}

fn extend<S: State + Clone, SP: StateSpace<StateType = S>, G: Goal<S>>(
    tree: &mut Vec<Node<S>>,
    q_target: &S,
    pd: &ProblemDefinition<S, SP, G>,
    vc: &Arc<dyn StateValidityChecker<S>>,
    max_distance: f64,
) -> Option<(ExtendResult, usize)> {
    let mut nearest_node_index = 0;
    let mut min_dist = pd.space.distance(&tree[0].state, q_target);
    for (i, node) in tree.iter().enumerate().skip(1) {
        let dist = pd.space.distance(&node.state, q_target);
        if dist < min_dist {
            min_dist = dist;
            nearest_node_index = i;
        }
    }

    let q_near = tree[nearest_node_index].state.clone();
    let mut q_new = q_near.clone();
    let result = if min_dist > max_distance {
        let t = max_distance / min_dist;
        pd.space.interpolate(&q_near, q_target, t, &mut q_new);
        ExtendResult::Advanced
    } else {
        q_new = q_target.clone();
        ExtendResult::Reached
    };

    if check_motion(&q_near, &q_new, pd, vc, max_distance) {
        let new_node_idx = tree.len();
        tree.push(Node {
            state: q_new,
            parent_index: Some(nearest_node_index),
        });
        Some((result, new_node_idx))
    } else {
        None
    }
}

fn check_motion<S: State + Clone, SP: StateSpace<StateType = S>, G: Goal<S>>(
    from: &S,
    to: &S,
    pd: &ProblemDefinition<S, SP, G>,
    vc: &Arc<dyn StateValidityChecker<S>>,
    max_distance: f64,
) -> bool {
    let space = &pd.space;
    let dist = space.distance(from, to);
    let num_steps = (dist / (max_distance * 0.1)).ceil() as usize;

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
}
