
import pytest
import math
import random

from oxmpl_py.base import CompoundState, CompoundStateSpace, RealVectorStateSpace, SO2StateSpace, ProblemDefinition, RealVectorState, SO2State
from oxmpl_py.geometric import RRT


class CompoundCircularGoal:
    def __init__(self, space: CompoundStateSpace, x: float, y: float, radius: float):
        self.space = space
        self.target = CompoundState([RealVectorState([x, y]), SO2State(0.0)])
        self.radius = radius
        self.rng = random.Random(123)

    def is_satisfied(self, state: CompoundState) -> bool:
        return self.space.distance(self.target, state) <= self.radius

    def sample_goal(self) -> CompoundState:
        angle = self.rng.uniform(0, 2 * math.pi)
        radius = self.radius * math.sqrt(self.rng.uniform(0, 1))

        x = self.target.components[0].values[0] + radius * math.cos(angle)
        y = self.target.components[0].values[1] + radius * math.sin(angle)

        # For simplicity, the SO2 part of the goal is fixed
        so2_value = self.rng.uniform(0, 2 * math.pi)

        return CompoundState([RealVectorState([x, y]), SO2State(so2_value)])


def is_state_valid(state: CompoundState) -> bool:
    rv_state = state.components[0]
    x, y = rv_state.values

    wall_x_pos = 5.0
    wall_y_min = 2.0
    wall_y_max = 8.0
    wall_thickness = 0.5

    is_in_wall = (
        x >= wall_x_pos - wall_thickness / 2.0
        and x <= wall_x_pos + wall_thickness / 2.0
        and y >= wall_y_min
        and y <= wall_y_max
    )

    return not is_in_wall


def test_rrt_finds_path_in_css():
    rv_space = RealVectorStateSpace(dimension=2, bounds=[(0.0, 10.0), (0.0, 10.0)])
    so2_space = SO2StateSpace()
    space = CompoundStateSpace([rv_space, so2_space], weights=[1.0, 0.5])

    start_state = CompoundState([RealVectorState([1.0, 5.0]), SO2State(0.0)])
    goal_region = CompoundCircularGoal(space, x=9.0, y=5.0, radius=0.5)

    problem_def = ProblemDefinition.from_compound(space, start_state, goal_region)

    planner = RRT(max_distance=0.5, goal_bias=0.05, problem_definition=problem_def)

    planner.setup(is_state_valid)

    print("\nAttempting to solve planning problem...")
    try:
        path = planner.solve(timeout_secs=5.0)
        print(f"Solution found with {len(path.states)} states.")
    except Exception as e:
        pytest.fail(
            f"Planner failed to find a solution when one should exist. Error: {e}"
        )

    assert len(path.states) > 1, "Path should contain at least a start and end state."

    path_start = path.states[0]
    assert space.distance(path_start, start_state) < 1e-9, (
        "Path must start at the start state."
    )

    path_end = path.states[-1]
    assert goal_region.is_satisfied(path_end), "Path must end inside the goal region."

    for state in path.states:
        assert is_state_valid(state), f"Path contains an invalid state: {state.values}"

    print("Path validation successful!")
