# Python API Reference

The `oxmpl-py` package provides Python bindings for the core OxMPL functionality.

## `oxmpl_py.base`

This module contains the fundamental components for defining a motion planning problem.

### States
These classes represent a single point or configuration in a state space.

#### `RealVectorState`
A state in an N-dimensional Euclidean space ($R^n$).
- `__init__(values: List[float])`
- `values: List[float]` (read-only)

#### `SO2State`
A state representing a 2D rotation ($[-\pi, \pi)$).
- `__init__(value: float)`
- `value: float` (read-only)

#### `SO3State`
A state representing a 3D rotation (Quaternion).
- `__init__(x: float, y: float, z: float, w: float)`
- `identity() -> SO3State` (static)
- `x: float`, `y: float`, `z: float`, `w: float` (read-only)

#### `SE2State`
A state representing a 2D rigid body transformation ($x, y, yaw$). It is a composition of `RealVectorState` (for translation) and `SO2State` (for rotation).
- `__init__(x: float, y: float, yaw: float)`
- `x: float`, `y: float`, `yaw: float` (read-only)
- `translation: RealVectorState` (read-only)
- `rotation: SO2State` (read-only)

#### `SE3State`
A state representing a 3D rigid body transformation ($x, y, z, rotation$). It is a composition of `RealVectorState` and `SO3State`.
- `__init__(x: float, y: float, z: float, rotation: SO3State)`
- `x: float`, `y: float`, `z: float` (read-only)
- `translation: RealVectorState` (read-only)
- `rotation: SO3State` (read-only)

#### `CompoundState`
A state composed of one or more other state objects.
- `__init__(components: List[State])`
- `components: List[State]` (read-only)
- `__len__() -> int`: Returns the number of component states.

---

### State Spaces
These classes define the planning space, including its dimensions, boundaries, and distance metrics.

#### `RealVectorStateSpace`
Defines an N-dimensional space for `RealVectorState` instances.
- `__init__(dimension: int, bounds: Optional[List[Tuple[float, float]]] = None)`
- `distance(s1: RealVectorState, s2: RealVectorState) -> float`
- `get_maximum_extent() -> float`
- `set_longest_valid_segment_fraction(fraction: float)`

#### `SO2StateSpace`
Defines a space for `SO2State` instances.
- `__init__(bounds: Optional[Tuple[float, float]] = None)`
- `distance(s1: SO2State, s2: SO2State) -> float`
- `get_maximum_extent() -> float`
- `set_longest_valid_segment_fraction(fraction: float)`

#### `SO3StateSpace`
Defines a space for `SO3State` instances.
- `__init__(bounds: Optional[Tuple[SO3State, float]] = None)`
    - `bounds`: A tuple containing a center rotation and the max angle from that center.
- `distance(s1: SO3State, s2: SO3State) -> float`
- `get_maximum_extent() -> float`
- `set_longest_valid_segment_fraction(fraction: float)`

#### `SE2StateSpace`
Defines the state space for 2D rigid body motion (SE(2)).
- `__init__(weight: float, bounds: Optional[List[Tuple[float, float]]] = None)`
    - `bounds`: min/max for x and y.
- `distance(state1: SE2State, state2: SE2State) -> float`

#### `SE3StateSpace`
Defines the state space for 3D rigid body motion (SE(3)).
- `__init__(weight: float, bounds: Optional[List[Tuple[float, float]]] = None)`
    - `bounds`: min/max for x, y, and z.
- `distance(state1: SE3State, state2: SE3State) -> float`

#### `CompoundStateSpace`
Defines a space composed of multiple, weighted subspaces.
- `__init__(subspaces: List[StateSpace], weights: List[float])`
- `distance(state1: CompoundState, state2: CompoundState) -> float`

---

### Planning Components

#### `Goal`
A user-defined Python class that specifies the goal condition for the planner. It must implement the following methods:
- `is_satisfied(state: State) -> bool`: Returns true if the state satisfies the goal.
- `distance_goal(state: State) -> float`: The distance from a state to the goal region.
- `sample_goal() -> State`: Returns a state sampled from the goal region.

#### `ProblemDefinition`
Encapsulates all components of a motion planning problem.
- `from_real_vector(space: RealVectorStateSpace, start: RealVectorState, goal: Goal) -> ProblemDefinition` (classmethod)
- `from_so2(space: SO2StateSpace, start: SO2State, goal: Goal) -> ProblemDefinition` (classmethod)
- `from_so3(space: SO3StateSpace, start: SO3State, goal: Goal) -> ProblemDefinition` (classmethod)
- `from_se2(space: SE2StateSpace, start: SE2State, goal: Goal) -> ProblemDefinition` (classmethod)
- `from_se3(space: SE3StateSpace, start: SE3State, goal: Goal) -> ProblemDefinition` (classmethod)
- `from_compound(space: CompoundStateSpace, start: CompoundState, goal: Goal) -> ProblemDefinition` (classmethod)

#### `PlannerConfig`
General configuration for planners.
- `__init__(seed: Optional[int] = None)`
- `seed: Optional[int]` (read-only)

#### `Path`
A sequence of states representing a solution path.
- `from_..._states(...)`: Static methods to create a Path from a list of specific state types.
- `states: List[State]` (read-only): Returns the list of states in the path.
- `__len__() -> int`

## `oxmpl_py.geometric`
This module contains the geometric planner implementations.

### `RRT`
Rapidly-exploring Random Tree.
- `__init__(max_distance: float, goal_bias: float, problem_definition: ProblemDefinition, planner_config: PlannerConfig)`
- `setup(validity_checker: Callable[[State], bool])`
- `solve(timeout_secs: float) -> Path`

### `RRTConnect`
Bi-directional RRT algorithm.
- `__init__(max_distance: float, goal_bias: float, problem_definition: ProblemDefinition, planner_config: PlannerConfig)`
- `setup(validity_checker: Callable[[State], bool])`
- `solve(timeout_secs: float) -> Path`

### `RRTStar`
RRT* (Optimal RRT) algorithm.
- `__init__(max_distance: float, goal_bias: float, search_radius: float, problem_definition: ProblemDefinition, planner_config: PlannerConfig)`
- `setup(validity_checker: Callable[[State], bool])`
- `solve(timeout_secs: float) -> Path`

### `PRM`
Probabilistic RoadMap.
- `__init__(timeout: float, connection_radius: float, problem_definition: ProblemDefinition, planner_config: PlannerConfig)`
    - `timeout`: Time in seconds to spend building the roadmap.
- `setup(validity_checker: Callable[[State], bool])`
- `construct_roadmap()`
- `solve(timeout_secs: float) -> Path`
