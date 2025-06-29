import pytest
import math
import random

from oxmpl_py.base import RealVectorState, RealVectorStateSpace, ProblemDefinition
from oxmpl_py.geometric.planners import RRT

class CircularGoal:
    """
    A Python class that defines a goal region for the planner.
    The Rust binding code will call these methods.
    """
    def __init__(self, space: RealVectorStateSpace, x: float, y: float, radius: float):
        self.space = space
        self.target = RealVectorState([x, y])
        self.radius = radius
        self.rng = random.Random(123)

    def is_satisfied(self, state: RealVectorState) -> bool:
        """Checks if a state is within the goal region."""
        return self.space.distance(self.target, state) <= self.radius

    def sample_goal(self) -> RealVectorState:
        """Generates a random state from within the goal region."""
        angle = self.rng.uniform(0, 2 * math.pi)
        radius = self.radius * math.sqrt(self.rng.uniform(0, 1))
        
        x = self.target.values[0] + radius * math.cos(angle)
        y = self.target.values[1] + radius * math.sin(angle)
        return RealVectorState([x, y])

def is_state_valid(state: RealVectorState) -> bool:
    """
    A simple Python function to act as a state validity checker.
    This function defines a vertical wall obstacle in the middle of the space.
    """
    x, y = state.values
    
    wall_x_pos = 5.0
    wall_y_min = 2.0
    wall_y_max = 8.0
    wall_thickness = 0.5
    
    is_in_wall = (x >= wall_x_pos - wall_thickness / 2.0 and
                  x <= wall_x_pos + wall_thickness / 2.0 and
                  y >= wall_y_min and
                  y <= wall_y_max)
                  
    return not is_in_wall

def test_rrt_problem_with_wall():
    """
    An integration test that sets up a planning problem and verifies
    that the RRT planner can find a valid solution.
    """
    # DEFINE THE STATE SPACE
    # A 10x10 2D world
    space = RealVectorStateSpace(dimension=2, bounds=[(0.0, 10.0), (0.0, 10.0)])

    # DEFINE THE PROBLEM
    start_state = RealVectorState([1.0, 5.0])
    goal_region = CircularGoal(space, x=9.0, y=5.0, radius=0.5)

    problem_def = ProblemDefinition(space, start_state, goal_region)

    # CREATE AND SETUP THE PLANNER
    planner = RRT(max_distance=0.5, goal_bias=0.05)

    planner.setup(problem_def, is_state_valid)

    # SOLVE THE PROBLEM
    print("\nAttempting to solve planning problem...")
    try:
        path = planner.solve(timeout_secs=5.0)
        print(f"Solution found with {len(path.states)} states.")
    except Exception as e:
        pytest.fail(f"Planner failed to find a solution when one should exist. Error: {e}")

    # VALIDATE THE SOLUTION PATH
    assert len(path.states) > 1, "Path should contain at least a start and end state."

    path_start = path.states[0]
    assert space.distance(path_start, start_state) < 1e-9, "Path must start at the start state."

    path_end = path.states[-1]
    assert goal_region.is_satisfied(path_end), "Path must end inside the goal region."

    for state in path.states:
        assert is_state_valid(state), f"Path contains an invalid state: {state.values}"
    
    print("Path validation successful!")
