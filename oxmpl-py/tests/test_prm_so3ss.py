import pytest
import math
import random

from oxmpl_py.base import SO3State, SO3StateSpace, ProblemDefinition
from oxmpl_py.geometric import PRM


def quaternion_from_axis_angle(
    axis: tuple[float, float, float], angle: float
) -> SO3State:
    half_angle = angle * 0.5
    s = math.sin(half_angle)
    c = math.cos(half_angle)
    return SO3State(x=axis[0] * s, y=axis[1] * s, z=axis[2] * s, w=c)


class QuaternionGoalRegion:
    def __init__(self, space: SO3StateSpace, target: SO3State, radius: float):
        self.space = space
        self.target = target
        self.radius = radius
        self.rng = random.Random(123)

    def is_satisfied(self, state: SO3State) -> bool:
        return self.space.distance(self.target, state) <= self.radius

    def sample_goal(self) -> SO3State:
        while True:
            x, y, z, w = [self.rng.uniform(-1.0, 1.0) for _ in range(4)]
            norm_sq = x * x + y * y + z * z + w * w
            if 1e-9 < norm_sq < 1.0:
                norm = math.sqrt(norm_sq)
                random_quat = SO3State(x / norm, y / norm, z / norm, w / norm)
                if self.is_satisfied(random_quat):
                    return random_quat


# A global instance of the space is needed for the validity checker function
# because Python functions can't have member variables like classes do.
FORBIDDEN_ZONE_SPACE = SO3StateSpace()


def is_rotation_valid(state: SO3State) -> bool:
    forbidden_center = SO3State.identity()
    forbidden_radius = math.radians(44.9)

    distance_from_center = FORBIDDEN_ZONE_SPACE.distance(forbidden_center, state)

    return distance_from_center > forbidden_radius


def test_prm_finds_path_in_so3ss():
    space = SO3StateSpace()
    global FORBIDDEN_ZONE_SPACE
    FORBIDDEN_ZONE_SPACE = space

    start_state = quaternion_from_axis_angle(axis=(0.0, 1.0, 0.0), angle=math.pi / 2.0)
    goal_target = quaternion_from_axis_angle(axis=(0.0, 1.0, 0.0), angle=-math.pi / 2.0)
    goal_region = QuaternionGoalRegion(
        space, target=goal_target, radius=math.radians(10.0)
    )

    problem_def = ProblemDefinition.from_so3(space, start_state, goal_region)

    planner = PRM(timeout=5.0, connection_radius=0.5, problem_definition=problem_def)
    planner.setup(is_rotation_valid)
    planner.construct_roadmap()

    print("\nAttempting to solve SO(3) planning problem...")
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
        assert is_rotation_valid(state), (
            f"Path contains an invalid state at index {i}: {state}"
        )

    print("Path validation successful!")
