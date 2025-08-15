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

// A helper struct to build the tree. Each node stores its state and the index of its parent in
// the. For RRT* you also need to know the cost to get to the node.
#[derive(Clone)]
struct Node<S: State> {
    state: S,
    parent_index: Option<usize>,
    cost: f64,
}

/// An implementation of the RRT* (RRT-star) algorithm.
///
/// RRT* is a sampling-based algorithm that is asymptotically optimal, meaning it converges to the
/// shortest possible path as more samples are added. It extends the basic RRT algorithm with two
/// key features:
///
/// 1.  **"Choose Parent"**: When a new node is created, it searches for a parent among its
///     neighbors that will give it the lowest-cost path from the start.
/// 2.  **"Rewire"**: After a new node is added, it checks if it can provide a shorter path for any
///     of its neighbors, and if so, it updates their parent connections.
pub struct RRTStar<S: State, SP: StateSpace<StateType = S>, G: Goal<S>> {
    /// The maximum distance between nodes in the tree. This is the "step size".
    pub max_distance: f64,
    /// The probability of sampling the goal region instead of the whole space (e.g., 0.05 for 5%).
    pub goal_bias: f64,
    /// The radius to search for neighbors during the "Choose Parent" and "Rewire" steps.
    pub search_radius: f64,

    problem_def: Option<Arc<ProblemDefinition<S, SP, G>>>,
    validity_checker: Option<Arc<dyn StateValidityChecker<S>>>,
    tree: Vec<Node<S>>,
}

impl<S, SP, G> RRTStar<S, SP, G>
where
    S: State,
    SP: StateSpace<StateType = S>,
    G: Goal<S> + GoalSampleableRegion<S>,
{
    /// Creates a new `RRTStar` planner with the specified parameters.
    ///
    /// # Parameters
    /// * `max_distance` - The maximum length of a single branch in the tree.
    /// * `goal_bias` - The probability (0.0 to 1.0) of sampling the goal.
    /// * `search_radius` - The radius for finding neighbors to optimize connections.
    pub fn new(max_distance: f64, goal_bias: f64, search_radius: f64) -> Self {
        RRTStar {
            max_distance,
            goal_bias,
            search_radius,
            problem_def: None,
            validity_checker: None,
            tree: Vec::new(),
        }
    }

    fn check_motion(&self, from: &S, to: &S) -> bool {
        // We need access to the space and checker from our stored setup info.
        if let (Some(pd), Some(vc)) = (&self.problem_def, &self.validity_checker) {
            let space = &pd.space;

            // Determine the number of steps to check based on distance and resolution.
            // A simple approach: one check per unit of distance (or a fraction thereof).
            let dist = space.distance(from, to);
            let num_steps =
                (dist / (space.get_longest_valid_segment_length() * 0.1)).ceil() as usize;

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

    /// Calculates the cost to reach `current_node` if it were parented by `neighbour_node`.
    fn cost(&self, current_node: &Node<S>, neighbour_node: &Node<S>) -> f64 {
        if let Some(pd) = &self.problem_def {
            neighbour_node.cost
                + pd.space
                    .distance(&current_node.state, &neighbour_node.state)
        } else {
            f64::INFINITY
        }
    }

    /// Finds all nodes in the tree that are within the `search_radius` of a given node.
    ///
    /// This is a simple linear scan;
    /// TODO: Consider using kd-trees.
    fn find_neighbours(&self, node: &Node<S>) -> Vec<usize> {
        let mut neighbours: Vec<usize> = Vec::new();
        if let Some(pd) = &self.problem_def {
            for i in 0..self.tree.len() {
                if pd.space.distance(&node.state, &self.tree[i].state) < self.search_radius {
                    neighbours.push(i);
                }
            }
        }
        neighbours
    }

    fn reconstruct_path(&self, start_node_idx: usize) -> Path<S> {
        let mut path_states = Vec::new();
        let mut current_index = Some(start_node_idx);
        while let Some(index) = current_index {
            path_states.push(self.tree[index].state.clone());
            current_index = self.tree[index].parent_index;
        }
        path_states.reverse();

        Path(path_states)
    }
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
        self.problem_def = Some(problem_def);
        self.validity_checker = Some(validity_checker);
        self.tree.clear();

        // Initialise the tree with the start state.
        let start_state = self.problem_def.as_ref().unwrap().start_states[0].clone();
        let start_node = Node {
            state: start_state,
            parent_index: None,
            cost: 0.0,
        };
        self.tree.push(start_node);
    }

    fn solve(&mut self, timeout: Duration) -> Result<Path<S>, PlanningError> {
        let pd = self
            .problem_def
            .as_ref()
            .ok_or(PlanningError::PlannerUninitialised)?;
        let goal = &pd.goal;

        let start_time = Instant::now();
        let mut rng = rand::rng();

        // Main Loop
        loop {
            // 1. Check for timeout
            if start_time.elapsed() > timeout {
                return Err(PlanningError::Timeout);
            }

            // 2. Sample a state (q_rand)
            let q_rand = if rng.random_bool(self.goal_bias) {
                // TODO: assume sample_goal can't fail here for simplicity, but a real
                // implementation would handle the Result.
                goal.sample_goal(&mut rng).unwrap()
            } else {
                // TODO: assume uniform sampling can't fail if bounds are set correctly.
                pd.space.sample_uniform(&mut rng).unwrap()
            };

            // 3. Find the nearest node in the tree (q_near)
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

            // 4. Steer from q_near towards q_rand to get q_new
            let mut q_new = q_near.clone();
            if min_dist > self.max_distance {
                let t = self.max_distance / min_dist;
                pd.space.interpolate(q_near, &q_rand, t, &mut q_new);
            } else {
                q_new = q_rand;
            }

            // 5. Check if the motion to q_new is valid
            if !self.check_motion(q_near, &q_new) {
                continue;
            }

            let temp_node = Node {
                state: q_new.clone(),
                parent_index: None,
                cost: 0.0,
            };
            let neighbours: Vec<usize> = self.find_neighbours(&temp_node);

            // 6. Choose parent
            let mut best_parent_index = nearest_node_index;
            let q_near_node = &self.tree[nearest_node_index];
            let mut min_cost = self.cost(&temp_node, q_near_node);

            // Iterate through neighbors to find a cheaper path. If a neighbor offers a cheaper
            // path and the motion from that neighbor is collision-free we have found a new,
            // better parent.
            for &neighbour_idx in &neighbours {
                let neighbour_node = &self.tree[neighbour_idx];
                let cost_via_neighbour = self.cost(&temp_node, neighbour_node);

                if cost_via_neighbour < min_cost && self.check_motion(&neighbour_node.state, &q_new)
                {
                    min_cost = cost_via_neighbour;
                    best_parent_index = neighbour_idx;
                }
            }

            // 7. Add the new node to the tree with the optimal parent and cost.
            let new_node = Node {
                state: q_new.clone(),
                parent_index: Some(best_parent_index),
                cost: min_cost,
            };
            self.tree.push(new_node);
            let new_node_index = self.tree.len() - 1;

            // 8. Rewire tree
            for &neighbour_idx in &neighbours {
                let new_node_ref = &self.tree[new_node_index];
                let neighbour_node = &self.tree[neighbour_idx];

                // Don't try to rewire the new node's own parent.
                if new_node_ref.parent_index == Some(neighbour_idx) {
                    continue;
                }

                // Calculate the cost to reach the neighbor if it were routed through the new
                // node. If the path through the new node is shorter and the motion is valid
                // update the neighbor's parent and cost. This is the "rewire" operation.
                let cost_via_new_node = self.cost(neighbour_node, new_node_ref);

                if cost_via_new_node < neighbour_node.cost
                    && self.check_motion(&new_node_ref.state, &neighbour_node.state)
                {
                    let mutable_neighbour_node = &mut self.tree[neighbour_idx];
                    mutable_neighbour_node.parent_index = Some(new_node_index);
                    mutable_neighbour_node.cost = cost_via_new_node;
                }
            }

            // 9. Check if the new node satisfies the goal
            if goal.is_satisfied(&q_new) {
                println!("Solution found after {} nodes.", self.tree.len());
                return Ok(self.reconstruct_path(self.tree.len() - 1));
            }
        }
    }
}
