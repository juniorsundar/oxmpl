import pytest
import math
import random

from oxmpl_py.base import SO2State, SO2StateSpace, ProblemDefinition
from oxmpl_py.geometric import RRTStar

class AngleGoalRegion:
    def __init__(self, space: SO2StateSpace, target_angle: float, radius: float):
        self.space = space
        self.target = SO2State(target_angle)
        self.radius = radius
        self.rng = random.Random(456)

    def is_satisfied(self, state: SO2State) -> bool:
        return self.space.distance(self.target, state) <= self.radius

    def sample_goal(self) -> SO2State:
        random_offset = self.rng.uniform(-self.radius, self.radius)
        return SO2State(self.target.value + random_offset)


def is_angle_valid(state: SO2State) -> bool:
    angle = state.value

    # Define the forbidden zone properties
    forbidden_min = -0.5
    forbidden_max = 0.5

    is_in_forbidden_zone = angle >= forbidden_min and angle <= forbidden_max

    return not is_in_forbidden_zone


def test_rrt_star_finds_path_in_so2ss():
    space = SO2StateSpace()

    start_state = SO2State(-math.pi / 2.0)  # Start at -90 degrees
    goal_region = AngleGoalRegion(space, target_angle=math.pi / 2.0, radius=0.1)

    problem_def = ProblemDefinition.from_so2(space, start_state, goal_region)

    planner = RRTStar(
        max_distance=0.5,
        goal_bias=0.05,
        search_radius=0.25,
        problem_definition=problem_def,
    )

    planner.setup(is_angle_valid)

    print("\nAttempting to solve SO(2) planning problem...")
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
        assert is_angle_valid(state), f"Path contains an invalid state: {state.value}"

    print("Path validation successful!")
