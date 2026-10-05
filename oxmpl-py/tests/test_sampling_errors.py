import pytest

from oxmpl_py.base import (
    PlannerConfig,
    ProblemDefinition,
    RealVectorState,
    RealVectorStateSpace,
)
from oxmpl_py.geometric import PRM, RRT, RRTConnect, RRTStar


class FixedGoal:
    def is_satisfied(self, state: RealVectorState) -> bool:
        return state.values == [1.0]

    def distance_goal(self, state: RealVectorState) -> float:
        return abs(state.values[0] - 1.0)

    def sample_goal(self) -> RealVectorState:
        # RRTConnect needs a successful goal-root sample before uniform sampling.
        return RealVectorState([1.0])


@pytest.fixture
def unbounded_problem():
    space = RealVectorStateSpace(dimension=1, bounds=None)
    start = RealVectorState([0.0])
    return ProblemDefinition.from_real_vector(space, start, FixedGoal())


def test_prm_surfaces_sampling_error(unbounded_problem):
    planner = PRM(
        timeout=1.0,
        connection_radius=0.5,
        problem_definition=unbounded_problem,
        planner_config=PlannerConfig(seed=42),
    )
    planner.setup(lambda _state: True)

    with pytest.raises(Exception, match=r"dimension 0 is unbounded"):
        planner.construct_roadmap()


def test_rrt_surfaces_sampling_error(unbounded_problem):
    planner = RRT(
        max_distance=0.5,
        goal_bias=0.0,
        problem_definition=unbounded_problem,
        planner_config=PlannerConfig(seed=42),
    )
    planner.setup(lambda _state: True)

    with pytest.raises(Exception, match=r"dimension 0 is unbounded"):
        planner.solve(timeout_secs=1.0)


def test_rrt_star_surfaces_sampling_error(unbounded_problem):
    planner = RRTStar(
        max_distance=0.5,
        goal_bias=0.0,
        search_radius=0.25,
        problem_definition=unbounded_problem,
        planner_config=PlannerConfig(seed=42),
    )
    planner.setup(lambda _state: True)

    with pytest.raises(Exception, match=r"dimension 0 is unbounded"):
        planner.solve(timeout_secs=1.0)


def test_rrt_connect_surfaces_sampling_error(unbounded_problem):
    planner = RRTConnect(
        max_distance=0.5,
        goal_bias=0.0,
        problem_definition=unbounded_problem,
        planner_config=PlannerConfig(seed=42),
    )
    planner.setup(lambda _state: True)

    with pytest.raises(Exception, match=r"dimension 0 is unbounded"):
        planner.solve(timeout_secs=1.0)
