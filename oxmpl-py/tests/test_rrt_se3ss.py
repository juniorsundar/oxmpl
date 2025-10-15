import pytest
import math
import random

from oxmpl_py.base import SE3State, SO3State, SE3StateSpace, ProblemDefinition
from oxmpl_py.geometric import RRT


class SE3GoalRegion:
    def __init__(self, space: SE3StateSpace, x: float, y: float, z: float, radius: float):
        self.space = space
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.radius = radius
        self.rng = random.Random(123)

    def is_satisfied(self, state: SE3State) -> bool:
        dx = state.x - self.target_x
        dy = state.y - self.target_y
        dz = state.z - self.target_z
        return math.sqrt(dx * dx + dy * dy + dz * dz) <= self.radius

    def sample_goal(self) -> SE3State:
        # Sample a random point inside a sphere
        u = self.rng.uniform(0, 1)
        v = self.rng.uniform(0, 1)
        theta = 2 * math.pi * u
        phi = math.acos(2 * v - 1)
        x = self.target_x + self.radius * math.sin(phi) * math.cos(theta)
        y = self.target_y + self.radius * math.sin(phi) * math.sin(theta)
        z = self.target_z + self.radius * math.cos(phi)

        # Sample a random rotation
        quat_w = self.rng.uniform(-1.0, 1.0)
        quat_x = self.rng.uniform(-1.0, 1.0)
        quat_y = self.rng.uniform(-1.0, 1.0)
        quat_z = self.rng.uniform(-1.0, 1.0)
        norm = math.sqrt(quat_w**2 + quat_x**2 + quat_y**2 + quat_z**2)
        rotation = SO3State(w=quat_w/norm, x=quat_x/norm, y=quat_y/norm, z=quat_z/norm)

        return SE3State(x, y, z, rotation)


def is_state_valid(state: SE3State) -> bool:
    x, y, z = state.x, state.y, state.z

    # Obstacle 1: A spherical obstacle
    obs1_x, obs1_y, obs1_z, obs1_rad = 3.0, 3.0, 3.0, 1.0
    if math.sqrt((x - obs1_x) ** 2 + (y - obs1_y) ** 2 + (z - obs1_z) ** 2) < obs1_rad:
        return False

    # Obstacle 2: A box obstacle
    obs2_x_min, obs2_x_max = 6.0, 8.0
    obs2_y_min, obs2_y_max = 1.0, 4.0
    obs2_z_min, obs2_z_max = 2.0, 5.0
    if (obs2_x_min <= x <= obs2_x_max and
        obs2_y_min <= y <= obs2_y_max and
        obs2_z_min <= z <= obs2_z_max):
        return False

    return True


def test_rrt_finds_path_in_se3ss():
    space = SE3StateSpace(weight=1.0, bounds=[(0.0, 10.0), (0.0, 10.0), (0.0, 10.0)])
    start_state = SE3State(1.0, 1.0, 1.0, SO3State(w=1.0, x=0.0, y=0.0, z=0.0))
    goal_region = SE3GoalRegion(space, x=9.0, y=9.0, z=9.0, radius=0.5)

    problem_def = ProblemDefinition.from_se3(space, start_state, goal_region)

    planner = RRT(problem_definition=problem_def, goal_bias=0.1, max_distance=0.5)
    planner.setup(is_state_valid)

    print("\nAttempting to solve SE(3) planning problem with RRT...")
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
        assert is_state_valid(
            state
        ), f"Path contains an invalid state at index {i}: ({state.x}, {state.y}, {state.z})"

    print("Path validation successful!")
