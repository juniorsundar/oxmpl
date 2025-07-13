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

    fn generate_random_configuration(&self, rng: &mut impl Rng) -> Result<S, PlanningError> {
        let pd = self
            .problem_def
            .as_ref()
            .ok_or(PlanningError::PlannerUninitialised)?;
        let goal = &pd.goal;

        let q_rand = if rng.random_bool(self.goal_bias) {
            goal.sample_goal(rng).unwrap()
        } else {
            pd.space.sample_uniform(rng).unwrap()
        };

        Ok(q_rand)
    }

    fn find_nearest_node_idx(
        &self,
        q_target: &S,
        tree: &[Node<S>],
    ) -> Result<(usize, f64), PlanningError> {
        let pd = self
            .problem_def
            .as_ref()
            .ok_or(PlanningError::PlannerUninitialised)?;
        let mut nearest_node_index = 0;
        let mut min_dist = pd.space.distance(&tree[0].state, q_target);

        for (i, node) in tree.iter().enumerate().skip(1) {
            let dist = pd.space.distance(&node.state, q_target);
            if dist < min_dist {
                min_dist = dist;
                nearest_node_index = i;
            }
        }
        Ok((nearest_node_index, min_dist))
    }

    fn extend_branch() {
        todo!()
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
        let pd = self
            .problem_def
            .as_ref()
            .ok_or(PlanningError::PlannerUninitialised)?;
        let goal = &pd.goal;

        let start_time = Instant::now();
        let mut rng = rand::rng();

        loop {
            if start_time.elapsed() > timeout {
                return Err(PlanningError::Timeout);
            }

            let q_rand = self.generate_random_configuration(&mut rng)?;

            let (nearest_node_index, min_dist) =
                self.find_nearest_node_idx(&q_rand, &self.tree_a)?;
            let q_near = &self.tree_a[nearest_node_index].state;

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
                let new_node_index = self.tree_a.len();
                self.tree_a.push(new_node);

                if goal.is_satisfied(&q_new) {
                    println!("Solution found after {} nodes.", self.tree_a.len());
                    let mut path_states = Vec::new();
                    let mut current_index = Some(new_node_index);
                    while let Some(index) = current_index {
                        path_states.push(self.tree_a[index].state.clone());
                        current_index = self.tree_a[index].parent_index;
                    }
                    path_states.reverse();
                    return Ok(Path(path_states));
                }
            }
        }
    }
}
