# Concepts
OxMPL follows the modular design principles of OMPL. Understanding these core concepts is key to using the library effectively.

## State Space (`StateSpace`)
The `StateSpace` defines the configuration space of the robot. Common spaces include:
- `RealVectorStateSpace`: For $R^n$ spaces (e.g., 2D or 3D positions).
- `SE2StateSpace` / `SE3StateSpace`: For rigid bodies in 2D or 3D.
- `SO2StateSpace` / `SO3StateSpace`: For rotations.

## State (`State`)
A `State` represents a single point in the `StateSpace`. In Rust, these are often specific types like `RealVectorState`.

## State Validity Checker (`StateValidityChecker`)
This is a user-defined component that determines if a given `State` is "valid" (e.g., collision-free, within joint limits).

## Goal (`Goal`)
A `Goal` defines the termination criteria for the planner. It can be a simple state, a region, or a complex condition.

## Problem Definition (`ProblemDefinition`)
This brings together the `StateSpace`, the start state(s), and the `Goal`. It defines what problem the planner is trying to solve.

## Planner (`Planner`)
The algorithm used to find a path from the start to the goal. Examples include `RRT`, `RRT*`, `PRM`, and `RRTConnect`.
