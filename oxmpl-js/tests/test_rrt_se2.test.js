import oxmpl from 'oxmpl-js';
import { describe, expect, test } from 'vitest';

class SE2Goal {
  constructor(space, x, y, radius) {
    this.space = space;
    this.target = [x, y];
    this.radius = radius;
  }
  isSatisfied(state) {
    const dx = state.x - this.target[0];
    const dy = state.y - this.target[1];
    return Math.sqrt(dx * dx + dy * dy) <= this.radius;
  }
  distanceGoal(state) {
    const dx = state.x - this.target[0];
    const dy = state.y - this.target[1];
    const dist = Math.sqrt(dx * dx + dy * dy);
    return Math.max(0, dist - this.radius);
  }
  sampleGoal() {
    return new oxmpl.base.SE2State(this.target[0], this.target[1], 0.0);
  }
}

describe('RRT SE2 Integration Tests', () => {
  test('RRT problem in SE2 with obstacle', () => {
    const bounds = [-5, 5, -5, 5, -Math.PI, Math.PI];
    const space = new oxmpl.base.SE2StateSpace(1.0, bounds);

    const startState = new oxmpl.base.SE2State(-2.0, 0.0, 0.0);
    const goalRegion = new SE2Goal(space, 2.0, 0.0, 0.5);
    const goal = new oxmpl.base.Goal(goalRegion);

    const problemDef = oxmpl.base.ProblemDefinition.fromSE2State(space, startState, goal);

    const validityCheckerFn = (state) => {
      const x = state.x;
      const y = state.y;
      return x * x + y * y > 0.25;
    };
    const validityChecker = new oxmpl.base.StateValidityChecker(validityCheckerFn);

    const planner_config = new oxmpl.base.PlannerConfig(0);
    const planner = new oxmpl.geometric.RRT(0.5, 0.1, problemDef, planner_config);
    planner.setup(validityChecker);

    console.log('Solving (SE2)...');
    const path = planner.solve(5.0);
    console.log(`Solution found with ${path.getLength()} states.`);

    const states = path.getStates();
    expect(states.length).toBeGreaterThan(1);
    expect(space.distance(states[0], startState)).toBeLessThan(1e-9);
    expect(goalRegion.isSatisfied(states[states.length - 1])).toBe(true);

    for (const state of states) {
      expect(validityCheckerFn(state)).toBe(true);
    }
    console.log('SE2 Path validation successful!');
  });
});
