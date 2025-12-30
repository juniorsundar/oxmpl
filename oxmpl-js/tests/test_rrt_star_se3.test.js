import oxmpl from 'oxmpl-js';
import { describe, expect, test } from 'vitest';

class SE3Goal {
  constructor(space, target, radius) {
    this.space = space;
    this.target = target;
    this.radius = radius;
  }
  isSatisfied(state) {
    return this.space.distance(state, this.target) <= this.radius;
  }
  distanceGoal(state) {
    return Math.max(0, this.space.distance(state, this.target) - this.radius);
  }
  sampleGoal() {
    return new oxmpl.base.SE3State(
      this.target.x,
      this.target.y,
      this.target.z,
      this.target.rotation
    );
  }
}

describe('RRT* SE3 Integration Tests', () => {
  test('RRT* problem in SE3 with obstacle', () => {
    const bounds = [-10, 10, -10, 10, -10, 10];
    const space = new oxmpl.base.SE3StateSpace(1.0, bounds);

    const startState = new oxmpl.base.SE3State(-5.0, 0.0, 0.0, oxmpl.base.SO3State.identity());
    const targetState = new oxmpl.base.SE3State(5.0, 0.0, 0.0, oxmpl.base.SO3State.identity());

    const goalRegion = new SE3Goal(space, targetState, 0.5);
    const goal = new oxmpl.base.Goal(goalRegion);

    const problemDef = oxmpl.base.ProblemDefinition.fromSE3State(space, startState, goal);

    const validityCheckerFn = (state) => {
      const x = state.x;
      const y = state.y;
      const z = state.z;
      return x * x + y * y + z * z > 1.0;
    };
    const validityChecker = new oxmpl.base.StateValidityChecker(validityCheckerFn);

    const planner_config = new oxmpl.base.PlannerConfig(0);
    const planner = new oxmpl.geometric.RRTStar(1.0, 0.1, 2.0, problemDef, planner_config);
    planner.setup(validityChecker);

    console.log('Solving (SE3 RRT*)...');
    const path = planner.solve(5.0);
    console.log(`Solution found with ${path.getLength()} states.`);

    const states = path.getStates();
    expect(states.length).toBeGreaterThan(1);
    expect(space.distance(states[0], startState)).toBeLessThan(1e-9);
    expect(goalRegion.isSatisfied(states[states.length - 1])).toBe(true);

    for (const state of states) {
      expect(validityCheckerFn(state)).toBe(true);
    }
    console.log('SE3 RRT* Path validation successful!');
  });
});
