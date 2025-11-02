import pytest
import math
import random

from oxmpl_py.base import SE2State, SE2StateSpace, ProblemDefinition, PlannerConfig
from oxmpl_py.geometric import RRTConnect


class SE2GoalRegion:
    def __init__(self, space: SE2StateSpace, x: float, y: float, radius: float):
        self.space = space
        self.target_x = x
        self.target_y = y
        self.radius = radius
        self.rng = random.Random(123)

    def is_satisfied(self, state: SE2State) -> bool:
        dx = state.x - self.target_x
        dy = state.y - self.target_y
        return math.sqrt(dx * dx + dy * dy) <= self.radius

    def sample_goal(self) -> SE2State:
        angle = self.rng.uniform(0, 2 * math.pi)
        radius = self.radius * math.sqrt(self.rng.uniform(0, 1))
        x = self.target_x + radius * math.cos(angle)
        y = self.target_y + radius * math.sin(angle)
        yaw = self.rng.uniform(-math.pi, math.pi)
        return SE2State(x, y, yaw)


def is_state_valid(state: SE2State) -> bool:
    x, y = state.x, state.y

    # Obstacle 1: A circular obstacle
    obs1_x, obs1_y, obs1_rad = 3.0, 3.0, 1.0
    if math.sqrt((x - obs1_x) ** 2 + (y - obs1_y) ** 2) < obs1_rad:
        return False

    # Obstacle 2: A rectangular obstacle
    obs2_x_min, obs2_x_max = 6.0, 8.0
    obs2_y_min, obs2_y_max = 1.0, 4.0
    if obs2_x_min <= x <= obs2_x_max and obs2_y_min <= y <= obs2_y_max:
        return False

    return True


def test_rrt_connect_finds_path_in_se2ss():
    space = SE2StateSpace(
        weight=1.0, bounds=[(0.0, 10.0), (0.0, 10.0), (-math.pi, math.pi)]
    )
    start_state = SE2State(1.0, 1.0, 0.0)
    goal_region = SE2GoalRegion(space, x=9.0, y=9.0, radius=0.5)

    problem_def = ProblemDefinition.from_se2(space, start_state, goal_region)
    planner_config = PlannerConfig(seed=1)

    planner = RRTConnect(
        problem_definition=problem_def,
        goal_bias=0.1,
        max_distance=0.5,
        planner_config=planner_config,
    )
    planner.setup(is_state_valid)

    print("\nAttempting to solve SE(2) planning problem with RRT-Connect...")
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

    for i, state in enumerate(path.states):
        assert is_state_valid(state), (
            f"Path contains an invalid state at index {i}: ({state.x}, {state.y}, {state.yaw})"
        )

    print("Path validation successful!")
