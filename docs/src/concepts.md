# Concepts
OxMPL follows the modular design principles of OMPL. Understanding these core concepts is key to using the library effectively.

## State Space (`StateSpace`)
The `StateSpace` defines the configuration space of the robot.

### Fixed Named Spaces
- `RealVectorStateSpace`: For $R^n$ spaces (e.g., 2D or 3D positions).
- `SE2StateSpace` / `SE3StateSpace`: For rigid bodies in 2D or 3D.
- `SO2StateSpace` / `SO3StateSpace`: For rotations.

### Compound State Space (`CompoundStateSpace`)
A `CompoundStateSpace` is the dynamic API for arbitrary ordered products of component spaces, each optionally weighted. Use it when your system does not map to a single fixed named space and is naturally modelled as a collection of simpler state spaces.

> **Note**: Fixed named spaces such as `SE2StateSpace` and `SE3StateSpace` use their dedicated APIs and are not valid compound subspaces.

## State (`State`)
A `State` represents a single point in a `StateSpace`.

### Fixed Named States
These correspond to fixed named spaces:
- `RealVectorState`: An $R^n$ point.
- `SE2State` / `SE3State`: A 2D or 3D rigid-body configuration.
- `SO2State` / `SO3State`: A 2D or 3D rotation.

### Compound State (`CompoundState`)
A `CompoundState` is a dynamic heterogeneous container holding an ordered collection of component states, one per component space in a `CompoundStateSpace`. Each component is accessed by index.

> **Note**: Fixed named states such as `SE2State` and `SE3State` use their dedicated APIs and are not valid `CompoundState` components.

## State Validity Checker (`StateValidityChecker`)
This is a user-defined component that determines if a given `State` is "valid" (e.g., collision-free, within joint limits).

## Goal (`Goal`)
A `Goal` defines the termination criteria for the planner. It can be a simple state, a region, or a complex condition.

## Problem Definition (`ProblemDefinition`)
This brings together the `StateSpace`, the start state(s), and the `Goal`. It defines what problem the planner is trying to solve.

## Planner (`Planner`)
The algorithm used to find a path from the start to the goal. Examples include `RRT`, `RRT*`, `PRM`, and `RRTConnect`.
