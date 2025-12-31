# JavaScript API Reference

The `oxmpl-js` package provides WASM-based bindings for JavaScript environments.

## `oxmpl.base`

This module contains the fundamental components for defining a motion planning problem.

### States
These classes represent a single point or configuration in a state space.

#### `RealVectorState`
A state in an N-dimensional Euclidean space ($R^n$).
- `constructor(values: number[])`
- `values: number[]` (read-only)

#### `SO2State`
A state representing a 2D rotation ($[-\pi, \pi)$).
- `constructor(value: number)`
- `value: number` (read-only)
- `normalise(): SO2State`

#### `SO3State`
A state representing a 3D rotation (Quaternion).
- `constructor(x: number, y: number, z: number, w: number)`
- `identity(): SO3State` (static)
- `x: number`, `y: number`, `z: number`, `w: number` (read-only)
- `normalise(): SO3State`

#### `SE2State`
A state representing a 2D rigid body transformation ($x, y, yaw$).
- `constructor(x: number, y: number, yaw: number)`
- `x: number`, `y: number`, `yaw: number` (read-only)
- `translation: RealVectorState` (read-only)
- `rotation: SO2State` (read-only)

#### `SE3State`
A state representing a 3D rigid body transformation ($x, y, z, rotation$).
- `constructor(x: number, y: number, z: number, rotation: SO3State)`
- `x: number`, `y: number`, `z: number` (read-only)
- `translation: RealVectorState` (read-only)
- `rotation: SO3State` (read-only)

#### `CompoundState`
A state composed of one or more other state objects.
- `componentCount: number` (read-only)
- `getComponent(index: number): State`

#### `CompoundStateBuilder`
A helper class to construct `CompoundState` instances.
- `constructor()`
- `addRealVectorState(state: RealVectorState)`
- `addSO2State(state: SO2State)`
- `addSO3State(state: SO3State)`
- `addSE2State(state: SE2State)`
- `addSE3State(state: SE3State)`
- `addCompoundState(state: CompoundState)`
- `build(): CompoundState`

---

### State Spaces
These classes define the planning space, including its dimensions, boundaries, and distance metrics.

#### `RealVectorStateSpace`
Defines an N-dimensional space for `RealVectorState` instances.
- `constructor(dimension: number, bounds: number[] | undefined)`
    - `bounds`: A flat array `[min1, max1, min2, max2, ...]`.
- `sample(): RealVectorState`
- `distance(state1: RealVectorState, state2: RealVectorState): number`
- `satisfiesBounds(state: RealVectorState): boolean`
- `enforceBounds(state: RealVectorState): RealVectorState`
- `interpolate(from: RealVectorState, to: RealVectorState, t: number): RealVectorState`
- `getDimension(): number`
- `getMaximumExtent(): number`
- `getLongestValidSegmentLength(): number`
- `setLongestValidLineSegmentFraction(fraction: number)`

#### `SO2StateSpace`
Defines a space for `SO2State` instances.
- `constructor(bounds: number[] | undefined)`
    - `bounds`: `[min, max]`.
- `sample(): SO2State`
- `distance(state1: SO2State, state2: SO2State): number`
- `satisfiesBounds(state: SO2State): boolean`
- `enforceBounds(state: SO2State): SO2State`
- `interpolate(from: SO2State, to: SO2State, t: number): SO2State`
- `getMaximumExtent(): number`
- `getLongestValidSegmentLength(): number`
- `setLongestValidLineSegmentFraction(fraction: number)`

#### `SO3StateSpace`
Defines a space for `SO3State` instances.
- `constructor(center: SO3State | undefined, maxAngle: number | undefined)`
- `sample(): SO3State`
- `distance(state1: SO3State, state2: SO3State): number`
- `satisfiesBounds(state: SO3State): boolean`
- `enforceBounds(state: SO3State): SO3State`
- `interpolate(from: SO3State, to: SO3State, t: number): SO3State`
- `getMaximumExtent(): number`
- `getLongestValidSegmentLength(): number`
- `setLongestValidLineSegmentFraction(fraction: number)`

#### `SE2StateSpace`
Defines the state space for 2D rigid body motion (SE(2)).
- `constructor(weight: number, bounds: number[] | undefined)`
    - `bounds`: `[x_min, x_max, y_min, y_max, yaw_min, yaw_max]`.
- `sample(): SE2State`
- `distance(state1: SE2State, state2: SE2State): number`
- `satisfiesBounds(state: SE2State): boolean`
- `enforceBounds(state: SE2State): SE2State`
- `interpolate(from: SE2State, to: SE2State, t: number): SE2State`
- `getLongestValidSegmentLength(): number`

#### `SE3StateSpace`
Defines the state space for 3D rigid body motion (SE(3)).
- `constructor(weight: number, bounds: number[] | undefined)`
    - `bounds`: `[x_min, x_max, y_min, y_max, z_min, z_max]`.
- `sample(): SE3State`
- `distance(state1: SE3State, state2: SE3State): number`
- `satisfiesBounds(state: SE3State): boolean`
- `enforceBounds(state: SE3State): SE3State`
- `interpolate(from: SE3State, to: SE3State, t: number): SE3State`
- `getLongestValidSegmentLength(): number`

#### `CompoundStateSpace`
Defines a space composed of multiple, weighted subspaces.
- `sample(): CompoundState`
- `distance(state1: CompoundState, state2: CompoundState): number`
- `satisfiesBounds(state: CompoundState): boolean`
- `enforceBounds(state: CompoundState): CompoundState`
- `interpolate(from: CompoundState, to: CompoundState, t: number): CompoundState`
- `getLongestValidSegmentLength(): number`

#### `CompoundStateSpaceBuilder`
A helper class to construct `CompoundStateSpace` instances.
- `constructor()`
- `addRealVectorStateSpace(space: RealVectorStateSpace, weight: number)`
- `addSO2StateSpace(space: SO2StateSpace, weight: number)`
- `addSO3StateSpace(space: SO3StateSpace, weight: number)`
- `addSE2StateSpace(space: SE2StateSpace, weight: number)`
- `addSE3StateSpace(space: SE3StateSpace, weight: number)`
- `addCompoundStateSpace(space: CompoundStateSpace, weight: number)`
- `build(): CompoundStateSpace`

---

### Planning Components

#### `Goal`
An interface that user-provided objects must satisfy to define the goal.
- `constructor(config: { isSatisfied: Function, distanceGoal: Function, sampleGoal: Function })`
    - `isSatisfied(state: State): boolean`
    - `distanceGoal(state: State): number`
    - `sampleGoal(): State`

#### `ProblemDefinition`
Encapsulates all components of a motion planning problem.
- `fromRealVectorState(space: RealVectorStateSpace, start: RealVectorState, goal: Goal): ProblemDefinition` (static)
- `fromSO2State(space: SO2StateSpace, start: SO2State, goal: Goal): ProblemDefinition` (static)
- `fromSO3State(space: SO3StateSpace, start: SO3State, goal: Goal): ProblemDefinition` (static)
- `fromSE2State(space: SE2StateSpace, start: SE2State, goal: Goal): ProblemDefinition` (static)
- `fromSE3State(space: SE3StateSpace, start: SE3State, goal: Goal): ProblemDefinition` (static)
- `fromCompoundState(space: CompoundStateSpace, start: CompoundState, goal: Goal): ProblemDefinition` (static)

#### `StateValidityChecker`
Wrapper for a user-defined function that checks if a state is valid (collision-free).
- `constructor(callback: (state: State) => boolean)`

#### `PlannerConfig`
General configuration for planners.
- `constructor(seed: number | undefined)`

#### `Path`
A sequence of states representing a solution path.
- `getStates(): State[]`
- `getLength(): number`

## `oxmpl.geometric`

### `RRT`
Rapidly-exploring Random Tree.
- `constructor(maxDistance: number, goalBias: number, problem: ProblemDefinition, config: PlannerConfig)`
- `setup(validityChecker: StateValidityChecker)`
- `solve(timeout: number): Path`

### `RRTConnect`
Bi-directional RRT algorithm.
- `constructor(maxDistance: number, goalBias: number, problem: ProblemDefinition, config: PlannerConfig)`
- `setup(validityChecker: StateValidityChecker)`
- `solve(timeout: number): Path`

### `RRTStar`
RRT* (Optimal RRT) algorithm.
- `constructor(maxDistance: number, goalBias: number, searchRadius: number, problem: ProblemDefinition, config: PlannerConfig)`
- `setup(validityChecker: StateValidityChecker)`
- `solve(timeout: number): Path`

### `PRM`
Probabilistic RoadMap.
- `constructor(timeout: number, connectionRadius: number, problem: ProblemDefinition, config: PlannerConfig)`
    - `timeout`: Time in seconds to spend building the roadmap.
- `setup(validityChecker: StateValidityChecker)`
- `constructRoadmap()`
- `solve(timeout: number): Path`
