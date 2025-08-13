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

// A helper struct to build the tree. Each node stores its state and the index of its parent in the
#[derive(Clone)]
struct Node<S: State> {
    state: S,
    parent_index: Option<usize>,
}

/// The result of an `extend` operation on a tree.
#[derive(PartialEq, Debug)]
enum ExtendResult {
    /// The tree was extended, but did not reach the target state.
    Advanced,
    /// The tree was extended and reached the target state exactly.
    Reached,
}

/// An implementation of the Rapidly-exploring Random Tree Connect (RRT-Connect) algorithm.
///
/// RRT-Connect is a bidirectional search algorithm that grows two trees, one from the start state
/// (`start_tree`) and one from a goal state (`goal_tree`). It is generally much faster at finding
/// solutions than the standard RRT algorithm, especially in open spaces.
///
/// # Algorithm Overview
/// 1. Initialize two trees: `start_tree` with the start state, and `goal_tree` with a state from
///    the goal region.
/// 2. Loop:
///    a. Sample a random state `q_rand`.
///    b. Select which tree to grow (usually the smaller one to keep them balanced). Let's call it
///    `tree_a` and the other `tree_b`.
///    c. Try to `extend` `tree_a` towards `q_rand` to create a new state `q_new`.
///    d. If the extension was successful (a new node was added), try to `connect` `tree_b` to `q_new`.
///    The `connect` operation repeatedly calls `extend` from `tree_b` towards `q_new` until it
///    reaches `q_new` or gets stuck.
///    e. If the `connect` operation successfully reaches `q_new`, the two trees have been joined,
///    and a solution path is found by combining the paths from the start and goal to the
///    connection point.
///
/// # Trait Bounds
///
/// To use this planner, the following trait bounds must be met:
/// - The `State` type (`S`) must be `Clone`.
/// - The `Goal` type (`G`) must implement `GoalSampleableRegion` to initialize the goal tree.
pub struct RRTConnect<S: State, SP: StateSpace<StateType = S>, G: Goal<S>> {
    /// The maximum distance between nodes in the tree. This is the "step size".
    pub max_distance: f64,
    /// The probability of sampling the goal region instead of the whole space (e.g., 0.05 for 5%).
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
    /// Creates a new `RRTConnect` planner with the specified parameters.
    ///
    /// # Parameters
    /// * `max_distance` - The maximum length of a single branch in the tree.
    /// * `goal_bias` - The probability (0.0 to 1.0) of sampling the goal.
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

    /// Helper function to extend a tree towards a target state.
    ///
    /// This function finds the node in the `tree` nearest to `q_target`. It then creates a new state
    /// `q_new` by moving from the nearest node towards `q_target` by a distance of at most
    /// `max_distance`. If the motion to `q_new` is valid, it adds `q_new` to the tree.
    ///
    /// Returns a tuple `(ExtendResult, usize)` on success, where `usize` is the index of the new node.
    /// Returns `None` if the motion was invalid.
    ///
    /// > [!WARNING]
    /// > This is associated function of RRTConnect struct because we are mutating the start and
    /// > goal trees inside > this. There were a lot of mutability/immutability issues and the
    /// > compiler was complaining.
    fn extend(
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

        if Self::check_motion(&q_near, &q_new, pd, vc) {
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

    /// An internal helper function to check if the motion between two states is valid.
    ///
    /// It works by discretizing the straight-line path between `from` and `to` into small steps and
    /// calling the `StateValidityChecker` on each intermediate state. If any intermediate state is
    /// invalid, the entire motion is considered invalid.
    fn check_motion(
        from: &S,
        to: &S,
        pd: &ProblemDefinition<S, SP, G>,
        vc: &Arc<dyn StateValidityChecker<S>>,
    ) -> bool {
        let space = &pd.space;
        let dist = space.distance(from, to);
        let num_steps = (dist / (space.get_longest_valid_segment_length() * 0.1)).ceil() as usize;

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
}

/// The main implementation of the Planner trait for RRTConnect.
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

        // Initialise the trees beginning from start and goal states.
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

        // Main loop
        loop {
            // 1. Check for timeout
            if start_time.elapsed() > timeout {
                return Err(PlanningError::Timeout);
            }

            // 2. Determine which tree to grow (tree_a) and which to connect to (tree_b). This
            //    balances the trees, which is more efficient.
            let (tree_a, tree_b, is_growing_start_tree) =
                if self.start_tree.len() <= self.goal_tree.len() {
                    (&mut self.start_tree, &mut self.goal_tree, true)
                } else {
                    (&mut self.goal_tree, &mut self.start_tree, false)
                };

            // 3. Sample a random target state `q_rand`, with goal biasing.
            // TODO: Handle sampling failures.
            let q_rand = if rng.random_bool(self.goal_bias) {
                goal.sample_goal(&mut rng).unwrap()
            } else {
                pd.space.sample_uniform(&mut rng).unwrap()
            };

            // 4. Try to extend tree_a towards q_rand.
            if let Some((_extend_result, new_node_idx_a)) =
                Self::extend(tree_a, &q_rand, pd, vc, self.max_distance)
            {
                let q_new = &tree_a[new_node_idx_a].state;

                // If growing the start tree, check if the new node is already in the goal.
                if is_growing_start_tree && goal.is_satisfied(q_new) {
                    println!("Solution found by start tree reaching goal directly.");
                    return Ok(self.reconstruct_path(&self.start_tree, new_node_idx_a));
                }

                // 5. Try to connect tree_b to the new state `q_new`.
                if let Some((connect_result, new_node_idx_b)) =
                    Self::extend(tree_b, q_new, pd, vc, self.max_distance)
                {
                    // 6. If the connection reached q_new, a solution is found.
                    if connect_result == ExtendResult::Reached {
                        println!(
                            "Solution found after {} total nodes.",
                            self.start_tree.len() + self.goal_tree.len()
                        );

                        // Identify which final node belongs to which tree.
                        let (start_idx, goal_idx) = if is_growing_start_tree {
                            (new_node_idx_a, new_node_idx_b)
                        } else {
                            (new_node_idx_b, new_node_idx_a)
                        };

                        // 7. Reconstruct the path from both trees and merge them.
                        let mut start_path = self.reconstruct_path(&self.start_tree, start_idx).0;
                        let mut goal_path = self.reconstruct_path(&self.goal_tree, goal_idx).0;

                        // The goal path is from goal to connection, so it needs to be reversed.
                        goal_path.reverse();
                        // Append the goal path (skipping the first element, which is the duplicate
                        // connection point) to the start path.
                        start_path.extend(goal_path.into_iter().skip(1));

                        return Ok(Path(start_path));
                    }
                }
            }
        }
    }
}
