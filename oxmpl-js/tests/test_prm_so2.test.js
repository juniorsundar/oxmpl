import oxmpl from 'oxmpl-js';
import { describe, expect, test } from 'vitest';

class AngleGoal {
  constructor(space, targetAngle, radius) {
    this.space = space;
    this.target = new oxmpl.base.SO2State(targetAngle);
    this.radius = radius;
  }
  isSatisfied(state) {
    return this.space.distance(state, this.target) <= this.radius;
  }
  distanceGoal(state) {
    return Math.max(0, this.space.distance(state, this.target) - this.radius);
  }
  sampleGoal() {
    return new oxmpl.base.SO2State(this.target.value);
  }
}

describe('PRM SO2 Integration Tests', () => {
  test('PRM problem in SO2', () => {
    const space = new oxmpl.base.SO2StateSpace();
    const startState = new oxmpl.base.SO2State(-Math.PI / 2.0);
    const goalRegion = new AngleGoal(space, Math.PI / 2.0, 0.2);
    const goal = new oxmpl.base.Goal(goalRegion);
    const problemDef = oxmpl.base.ProblemDefinition.fromSO2State(space, startState, goal);

    const validityCheckerFn = (state) => {
      return !(state.value > -0.5 && state.value < 0.5);
    };
    const validityChecker = new oxmpl.base.StateValidityChecker(validityCheckerFn);

    const planner_config = new oxmpl.base.PlannerConfig(0);
    const planner = new oxmpl.geometric.PRM(1.0, 1.0, problemDef, planner_config);
    planner.setup(validityChecker);
    planner.constructRoadmap();

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
