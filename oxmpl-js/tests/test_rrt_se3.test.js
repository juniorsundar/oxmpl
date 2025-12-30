import oxmpl from 'oxmpl-js';
import { describe, expect, test } from 'vitest';

class SE3Goal {
  constructor(space, target, radius) {
    this.space = space;
    this.target = target;
    this.radius = radius;
  }
  isSatisfied(state) {
    const dx = state.x - this.target.x;
    const dy = state.y - this.target.y;
    const dz = state.z - this.target.z;
    return Math.sqrt(dx * dx + dy * dy + dz * dz) <= this.radius;
  }
  distanceGoal(state) {
    const dx = state.x - this.target.x;
    const dy = state.y - this.target.y;
    const dz = state.z - this.target.z;
    const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
    return Math.max(0, dist - this.radius);
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

describe('RRT SE3 Integration Tests', () => {
  test('RRT problem in SE3', () => {
    const space = new oxmpl.base.SE3StateSpace(1.0, [0, 10, 0, 10, 0, 10]);
    const startState = new oxmpl.base.SE3State(1.0, 1.0, 1.0, oxmpl.base.SO3State.identity());
    const targetState = new oxmpl.base.SE3State(9.0, 9.0, 9.0, oxmpl.base.SO3State.identity());
    const goalRegion = new SE3Goal(space, targetState, 0.5);
    const goal = new oxmpl.base.Goal(goalRegion);
    const problemDef = oxmpl.base.ProblemDefinition.fromSE3State(space, startState, goal);

    const validityCheckerFn = (state) => {
      const { x, y, z } = state;
      const distSq1 = (x - 3) ** 2 + (y - 3) ** 2 + (z - 3) ** 2;
      if (distSq1 < 1) return false;
      if (x >= 6 && x <= 8 && y >= 1 && y <= 4 && z >= 2 && z <= 5) return false;
      return true;
    };
    const validityChecker = new oxmpl.base.StateValidityChecker(validityCheckerFn);

    const planner_config = new oxmpl.base.PlannerConfig(0);
    const planner = new oxmpl.geometric.RRT(1.0, 0.1, problemDef, planner_config);
    planner.setup(validityChecker);

    const path = planner.solve(5.0);
    const states = path.getStates();
    expect(states.length).toBeGreaterThan(1);
    expect(space.distance(states[0], startState)).toBeLessThan(1e-9);
    expect(goalRegion.isSatisfied(states[states.length - 1])).toBe(true);

    for (const state of states) {
      expect(validityCheckerFn(state)).toBe(true);
    }
  });
});
