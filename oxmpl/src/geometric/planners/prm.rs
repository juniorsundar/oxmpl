// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use std::{
    sync::Arc,
    time::{Duration, Instant},
};

use crate::base::{
    error::PlanningError,
    goal::{Goal, GoalSampleableRegion},
    planner::{Path, Planner},
    problem_definition::ProblemDefinition,
    space::StateSpace,
    state::State,
    validity::StateValidityChecker,
};

// A helper struct to build the tree. Each node stores its state and the index of its parent in the
// tree vector.
#[derive(Clone)]
struct Node<S: State> {
    state: S,
    edges: Vec<usize>,
}

pub struct PRM<S: State, SP: StateSpace<StateType = S>, G: Goal<S>> {
    pub num_samples: usize,
    pub connection_radius: f64,

    problem_def: Option<Arc<ProblemDefinition<S, SP, G>>>,
    validity_checker: Option<Arc<dyn StateValidityChecker<S>>>,
    roadmap: Vec<Node<S>>,
}

impl<S, SP, G> PRM<S, SP, G>
where
    S: State,
    SP: StateSpace<StateType = S>,
    G: Goal<S>,
{
    pub fn new(num_samples: usize, connection_radius: f64) -> Self {
        PRM {
            num_samples,
            connection_radius,
            problem_def: None,
            validity_checker: None,
            roadmap: Vec::new(),
        }
    }

    fn check_motion(&self, from: &S, to: &S) -> bool {
        // We need access to the space and checker from our stored setup info.
        if let (Some(pd), Some(vc)) = (&self.problem_def, &self.validity_checker) {
            let space = &pd.space;
            // Determine the number of steps to check based on distance and resolution.
            // A simple approach: one check per unit of distance (or a fraction thereof).
            let dist = space.distance(from, to);
            let num_steps = (dist / (1000. * 0.1)).ceil() as usize;

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

    // fn reconstruct_path(&self, start_node_idx: usize) -> Path<S> {
    //     let path_states = Vec::new();
    //     Path(path_states)
    // }
}

// The main implementation of the Planner trait for RRT.
impl<S, SP, G> Planner<S, SP, G> for PRM<S, SP, G>
where
    // RRT needs to clone states to store them in its tree.
    S: State + Clone,
    SP: StateSpace<StateType = S>,
    // For goal biasing, the Goal type must be sampleable.
    G: Goal<S> + GoalSampleableRegion<S>,
{
    fn setup(
        &mut self,
        problem_def: Arc<ProblemDefinition<S, SP, G>>,
        validity_checker: Arc<dyn StateValidityChecker<S>>,
    ) {
        self.problem_def = Some(problem_def);
        self.validity_checker = Some(validity_checker);
        self.roadmap.clear();
    }

    fn solve(&mut self, timeout: Duration) -> Result<Path<S>, PlanningError> {
        // Ensure setup has been called.
        let pd = self
            .problem_def
            .as_ref()
            .ok_or(PlanningError::PlannerUninitialised)?;
        let vc = self
            .validity_checker
            .as_ref()
            .ok_or(PlanningError::PlannerUninitialised)?;
        let goal = &pd.goal;

        let start_time = Instant::now();
        let mut rng = rand::rng();

        while self.roadmap.len() < self.num_samples {
            if start_time.elapsed() > timeout {
                return Err(PlanningError::Timeout);
            }

            let q_rand = pd.space.sample_uniform(&mut rng).unwrap();
            if vc.is_valid(&q_rand) {
                let new_node_idx = self.roadmap.len();
                let mut new_node = Node {
                    state: q_rand,
                    edges: Vec::new(),
                };

                for i in 0..self.roadmap.len() {
                    let neighbor = self.roadmap[i].clone();
                    if pd.space.distance(&new_node.state, &neighbor.state) < self.connection_radius
                        && self.check_motion(&new_node.state, &neighbor.state)
                    {
                        new_node.edges.push(i);
                        self.roadmap[i].edges.push(new_node_idx); // Neighbor
                    }
                }
                self.roadmap.push(new_node);
            }
        }
        println!(
            "PRM: Roadmap constructed with {} milestones.",
            self.roadmap.len()
        );

        let start_state = &pd.start_states[0];
        if !vc.is_valid(start_state) {
            return Err(PlanningError::InvalidStartState);
        }

        let mut start_connections = Vec::new();
        for i in 0..self.roadmap.len() {
            if pd.space.distance(start_state, &self.roadmap[i].state) < self.connection_radius
                && self.check_motion(start_state, &self.roadmap[i].state)
            {
                start_connections.push(i);
            }
        }

        let mut goal_indices = Vec::new();
        for i in 0..self.roadmap.len() {
            if goal.is_satisfied(&self.roadmap[i].state) {
                goal_indices.push(i);
            }
        }

        if start_connections.is_empty() || goal_indices.is_empty() {
            return Err(PlanningError::NoSolutionFound);
        }

        Err(PlanningError::NoSolutionFound)
    }
}
