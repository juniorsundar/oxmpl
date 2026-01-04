import math
import random
from oxmpl_py.base import (
    CompoundState,
    CompoundStateSpace,
    RealVectorStateSpace,
    SO2StateSpace,
    ProblemDefinition,
    RealVectorState,
    SO2State,
    PlannerConfig,
)
from oxmpl_py.geometric import RRT

class CompoundCircularGoal:
    def __init__(self, space: CompoundStateSpace, x: float, y: float, radius: float):
        self.space = space
        # Target: Position (x,y) and Orientation 0.0 (though orientation is ignored for goal satisfaction here)
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

        # Random orientation for the goal sample
        so2_value = self.rng.uniform(0, 2 * math.pi)

        return CompoundState([RealVectorState([x, y]), SO2State(so2_value)])

def is_state_valid(state: CompoundState) -> bool:
    """
    Checks if the state is valid.
    Invalid if the position is inside a box obstacle centered at (0, 0)
    with width 1.0 and height 4.0.
    """
    rv_state = state.components[0]
    x, y = rv_state.values

    # Obstacle definition
    x_min, x_max = -0.5, 0.5
    y_min, y_max = -2.0, 2.0

    is_in_wall = (
        x >= x_min and x <= x_max and
        y >= y_min and y <= y_max
    )

    return not is_in_wall

def main():
    # 1. Define State Space: R^2 x SO(2)
    rv_space = RealVectorStateSpace(dimension=2, bounds=[(-5.0, 5.0), (-5.0, 5.0)])
    so2_space = SO2StateSpace()

    # Combined space with weights
    space = CompoundStateSpace([rv_space, so2_space], weights=[1.0, 0.5])

    # 2. Define Start and Goal
    start_state = CompoundState([RealVectorState([-2.0, 0.0]), SO2State(0.0)])

    # Goal region around (2.0, 0.0) with radius 0.5
    goal_region = CompoundCircularGoal(space, x=2.0, y=0.0, radius=0.5)

    # 3. Create Problem Definition
    problem_def = ProblemDefinition.from_compound(space, start_state, goal_region)
    planner_config = PlannerConfig(seed=123)

    # 4. Setup Planner (RRT)
    planner = RRT(
        max_distance=0.5,
        goal_bias=0.1,
        problem_definition=problem_def,
        planner_config=planner_config,
    )

    planner.setup(is_state_valid)

    # 5. Solve
    print("Solving Compound State planning problem (Python)...")
    try:
        path = planner.solve(timeout_secs=5.0)
        print(f"Solution found with {len(path.states)} states.")

        if len(path.states) > 0:
            start = path.states[0]
            end = path.states[-1]
            s_rv = start.components[0].values
            s_so2 = start.components[1].value
            e_rv = end.components[0].values
            e_so2 = end.components[1].value

            print(f"Start: ({s_rv[0]:.2f}, {s_rv[1]:.2f}, {s_so2:.2f})")
            print(f"End:   ({e_rv[0]:.2f}, {e_rv[1]:.2f}, {e_so2:.2f})")

    except Exception as e:
        print(f"Planning failed: {e}")

if __name__ == "__main__":
    main()
