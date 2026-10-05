// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use std::{
    collections::{HashMap, VecDeque},
    sync::Arc,
};

use log::debug;
use rand::{rngs::StdRng, SeedableRng};

use crate::time::{Duration, Instant};

use crate::base::{
    error::PlanningError,
    goal::{Goal, GoalSampleableRegion},
    planner::{Path, Planner, PlannerConfig},
    problem_definition::ProblemDefinition,
    space::StateSpace,
    state::State,
    validity::StateValidityChecker,
};

/// Represents a node (or "milestone") in the probabilistic roadmap.
#[derive(Clone)]
pub struct Node<S: State> {
    /// The state associated with this node.
    state: S,
    /// A list of indices pointing to other connected nodes in the roadmap.
    edges: Vec<usize>,
}

/// An implementation of the Probabilistic Roadmap (PRM) algorithm.
///
/// PRM is a multi-query, sampling-based algorithm that is particularly effective in static
/// environments. It works by first constructing a "roadmap" graph of valid states and then
/// querying this graph to find paths.
///
/// # Algorithm Overview
///
/// 1.  **Construction Phase**:
///     a. Sample a number of states randomly from the state space.
///     b. For each valid sample, find all nearby nodes already in the roadmap.
///     c. If a valid, collision-free motion exists between the new sample and a neighbor, add an
///     edge connecting them in the roadmap.
/// 2.  **Query Phase**:
///     a. Connect the start and goal states to the roadmap.
///     b. Use a graph search algorithm (in this case, Breadth-First Search) to find a path on the
///     roadmap from the start to the goal.
pub struct PRM<S: State, SP: StateSpace<StateType = S>, G: Goal<S>> {
    /// The time allocated for roadmap construction, in seconds.
    pub timeout: f64,
    /// The radius within which to search for neighbors to connect to a new sample.
    pub connection_radius: f64,

    problem_def: Option<Arc<ProblemDefinition<S, SP, G>>>,
    validity_checker: Option<Arc<dyn StateValidityChecker<S>>>,
    roadmap: Vec<Node<S>>,
    rng: Option<Box<StdRng>>,
}

impl<S, SP, G> PRM<S, SP, G>
where
    S: State + Clone,
    SP: StateSpace<StateType = S>,
    G: Goal<S>,
{
    /// Creates a new `PRM` planner with the specified parameters.
    ///
    /// # Parameters
    /// * `timeout` - The time in seconds to spend building the roadmap.
    /// * `connection_radius` - The radius for connecting new nodes to the roadmap.
    /// * `config` - The planner configuration, for planner-specific parameters.
    pub fn new(timeout: f64, connection_radius: f64, config: &PlannerConfig) -> Self {
        let rng = config.seed.map(|s| Box::new(StdRng::seed_from_u64(s)));
        PRM {
            timeout,
            connection_radius,
            problem_def: None,
            validity_checker: None,
            roadmap: Vec::new(),
            rng,
        }
    }

    /// Get private variable `roadmap` as a clone.
    /// TODO: Determine if this needs to be obtainable.
    pub fn get_roadmap(&self) -> Vec<Node<S>> {
        self.roadmap.clone()
    }

    /// Update ProblemDefinition. This is so that you can use an already sampled roadmap but just
    /// change the start and goal states.
    pub fn set_problem_definition(&mut self, pd: Arc<ProblemDefinition<S, SP, G>>) {
        self.problem_def = Some(pd);
    }

    /// Constructs the probabilistic roadmap.
    ///
    /// This method populates the roadmap by sampling states and connecting them until the
    /// specified timeout is reached.
    pub fn construct_roadmap(&mut self) -> Result<(), PlanningError> {
        let pd = self
            .problem_def
            .as_ref()
            .ok_or(PlanningError::PlannerUninitialised)?;
        let vc = self
            .validity_checker
            .as_ref()
            .ok_or(PlanningError::PlannerUninitialised)?;

        if !self.roadmap.is_empty() {
            debug!(
                "PRM: Roadmap already constructed with {} milestones.",
                self.roadmap.len()
            );

            return Ok(());
        }

        let start_time = Instant::now();
        loop {
            if start_time.elapsed().as_secs_f64() > self.timeout {
                break;
            }

            let q_rand = {
                let rng = self
                    .rng
                    .get_or_insert_with(|| Box::new(StdRng::from_os_rng()));
                match pd.space.sample_uniform(&mut *rng) {
                    Ok(q) => q,
                    Err(e) => {
                        self.roadmap.clear();
                        return Err(e.into());
                    }
                }
            };

            if vc.is_valid(&q_rand) {
                let mut new_node = Node {
                    state: q_rand.clone(),
                    edges: Vec::new(),
                };

                let mut to_update: Vec<usize> = Vec::new();

                for i in 0..self.roadmap.len() {
                    let other_state = self.roadmap[i].state.clone();
                    let dist = pd.space.distance(&q_rand, &other_state);
                    if dist < self.connection_radius && self.check_motion(&q_rand, &other_state) {
                        new_node.edges.push(i);
                        to_update.push(i);
                    }
                }

                let new_node_idx = self.roadmap.len();
                self.roadmap.push(new_node);

                for i in to_update {
                    self.roadmap[i].edges.push(new_node_idx);
                }
            }
        }
        debug!(
            "PRM: Roadmap constructed with {} milestones.",
            self.roadmap.len()
        );

        Ok(())
    }

    /// An internal helper function to check if the motion between two states is valid.
    ///
    /// It works by discretizing the straight-line path between `from` and `to` into small steps
    /// and calling the `StateValidityChecker` on each intermediate state. If any intermediate
    /// state is invalid, the entire motion is considered invalid.
    fn check_motion(&self, from: &S, to: &S) -> bool {
        // We need access to the space and checker from our stored setup info.
        if let (Some(pd), Some(vc)) = (&self.problem_def, &self.validity_checker) {
            let space = &pd.space;

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

    fn reconstruct_path(
        &self,
        start_state: &S,
        parent_map: HashMap<usize, Option<usize>>,
        goal_idx: usize,
    ) -> Path<S> {
        let mut path = vec![start_state.clone()];
        let mut current = goal_idx;
        let mut states = Vec::new();

        while let Some(parent) = parent_map[&current] {
            states.push(self.roadmap[current].state.clone());
            current = parent;
        }
        states.push(self.roadmap[current].state.clone());
        states.reverse();
        path.extend(states);

        Path(path)
    }
}

impl<S, SP, G> Planner<S, SP, G> for PRM<S, SP, G>
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

        if self.roadmap.is_empty() {
            return Err(PlanningError::UnsampledStateSpace);
        }

        let start_state = &pd.start_states[0];
        if !vc.is_valid(start_state) {
            return Err(PlanningError::InvalidStartState);
        }

        // Connect start state to the roadmap
        let mut start_connections = Vec::new();
        for i in 0..self.roadmap.len() {
            if pd.space.distance(start_state, &self.roadmap[i].state) < self.connection_radius
                && self.check_motion(start_state, &self.roadmap[i].state)
            {
                start_connections.push(i);
            }
        }

        // Find goal nodes in the roadmap
        let mut goal_indices = Vec::new();
        for i in 0..self.roadmap.len() {
            if goal.is_satisfied(&self.roadmap[i].state) {
                goal_indices.push(i);
            }
        }

        if start_connections.is_empty() || goal_indices.is_empty() {
            return Err(PlanningError::NoSolutionFound);
        }

        // Graph Search (Breadth-First Search)
        let mut queue: VecDeque<usize> = start_connections.clone().into_iter().collect();
        let mut parent_map: HashMap<usize, Option<usize>> = HashMap::new();
        let mut visited = vec![false; self.roadmap.len()];

        for idx in &start_connections {
            queue.push_back(*idx);
            parent_map.insert(*idx, None);
            visited[*idx] = true;
        }

        let mut goal_reached = None;

        let start_time = Instant::now();
        while let Some(current_idx) = queue.pop_front() {
            if start_time.elapsed() > timeout {
                return Err(PlanningError::Timeout);
            }

            if goal_indices.contains(&current_idx) {
                goal_reached = Some(current_idx);
                break;
            }

            for &neighbor_idx in &self.roadmap[current_idx].edges {
                if !visited[neighbor_idx] {
                    visited[neighbor_idx] = true;
                    parent_map.insert(neighbor_idx, Some(current_idx));
                    queue.push_back(neighbor_idx);
                }
            }
        }

        // If no goal was reached, no path exists
        let goal_node_idx = goal_reached.ok_or(PlanningError::NoSolutionFound)?;

        Ok(self.reconstruct_path(start_state, parent_map, goal_node_idx))
    }
}

#[cfg(test)]
mod tests {
    use std::cell::Cell;

    use super::*;
    use crate::base::{
        error::StateSamplingError,
        goal::GoalRegion,
        space::{RealVectorStateSpace, StateSpace},
        state::RealVectorState,
    };
    use rand::Rng;

    struct UnusedChecks;

    struct AlwaysValid;

    impl StateValidityChecker<RealVectorState> for AlwaysValid {
        fn is_valid(&self, _state: &RealVectorState) -> bool {
            true
        }
    }

    struct FailAfterOneSample {
        space: RealVectorStateSpace,
        first_sample: Cell<bool>,
    }

    impl StateSpace for FailAfterOneSample {
        type StateType = RealVectorState;

        fn distance(&self, state1: &RealVectorState, state2: &RealVectorState) -> f64 {
            self.space.distance(state1, state2)
        }

        fn interpolate(
            &self,
            from: &RealVectorState,
            to: &RealVectorState,
            t: f64,
            state: &mut RealVectorState,
        ) {
            self.space.interpolate(from, to, t, state);
        }

        fn enforce_bounds(&self, state: &mut RealVectorState) {
            self.space.enforce_bounds(state);
        }

        fn satisfies_bounds(&self, state: &RealVectorState) -> bool {
            self.space.satisfies_bounds(state)
        }

        fn sample_uniform(
            &self,
            rng: &mut impl Rng,
        ) -> Result<RealVectorState, StateSamplingError> {
            if self.first_sample.replace(false) {
                self.space.sample_uniform(rng)
            } else {
                Err(StateSamplingError::ZeroVolume)
            }
        }

        fn get_longest_valid_segment_length(&self) -> f64 {
            self.space.get_longest_valid_segment_length()
        }
    }

    impl Goal<RealVectorState> for UnusedChecks {
        fn is_satisfied(&self, _state: &RealVectorState) -> bool {
            panic!("Construction should not query the goal");
        }
    }

    impl GoalRegion<RealVectorState> for UnusedChecks {
        fn distance_goal(&self, _state: &RealVectorState) -> f64 {
            panic!("Construction should not query the goal");
        }
    }

    impl GoalSampleableRegion<RealVectorState> for UnusedChecks {
        fn sample_goal(&self, _rng: &mut impl Rng) -> Result<RealVectorState, StateSamplingError> {
            panic!("Construction should not sample the goal");
        }
    }

    impl StateValidityChecker<RealVectorState> for UnusedChecks {
        fn is_valid(&self, _state: &RealVectorState) -> bool {
            panic!("Sampling should fail before validity checking");
        }
    }

    #[test]
    fn test_failure_path_prm() {
        let rvss_unbound = Arc::new(RealVectorStateSpace::new(1, None).unwrap());
        let mut planner = PRM::new(30.0, 0.5, &PlannerConfig { seed: Some(42) });
        planner.set_problem_definition(Arc::new(ProblemDefinition {
            space: rvss_unbound,
            start_states: vec![RealVectorState { values: vec![0.0] }],
            goal: Arc::new(UnusedChecks),
        }));

        planner.validity_checker = Some(Arc::new(UnusedChecks));

        let result = planner.construct_roadmap();

        assert_eq!(
            result,
            Err(PlanningError::Sampling(
                StateSamplingError::UnboundedDimension { dimension_index: 0 }
            ))
        );
    }

    #[test]
    #[allow(clippy::arc_with_non_send_sync)]
    fn test_partial_roadmap() {
        let space = Arc::new(FailAfterOneSample {
            space: RealVectorStateSpace::new(1, Some(vec![(0.0, 1.0)])).unwrap(),
            first_sample: Cell::new(true),
        });
        let mut planner = PRM::new(30.0, 0.5, &PlannerConfig { seed: Some(42) });
        planner.setup(
            Arc::new(ProblemDefinition {
                space,
                start_states: vec![RealVectorState { values: vec![0.0] }],
                goal: Arc::new(UnusedChecks),
            }),
            Arc::new(AlwaysValid),
        );

        assert_eq!(
            planner.construct_roadmap(),
            Err(PlanningError::Sampling(StateSamplingError::ZeroVolume))
        );
        assert!(
            planner.get_roadmap().is_empty(),
            "Sampling failure left {} milestones in the roadmap",
            planner.get_roadmap().len()
        );
    }
}
