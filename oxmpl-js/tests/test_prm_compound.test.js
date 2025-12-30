import oxmpl from 'oxmpl-js';
import { describe, expect, test } from 'vitest';

class CompoundGoal {
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
    const tRV = this.target.getComponent(0);
    const tSO2 = this.target.getComponent(1);
    const rv = new oxmpl.base.RealVectorState(Array.from(tRV.values));
    const so2 = new oxmpl.base.SO2State(tSO2.value);
    const builder = new oxmpl.base.CompoundStateBuilder();
    builder.addRealVectorState(rv);
    builder.addSO2State(so2);
    return builder.build();
  }
}

describe('PRM Compound Integration Tests', () => {
  test('PRM problem in Compound Space', () => {
    // R2 + SO2
    const r2Space = new oxmpl.base.RealVectorStateSpace(2, [-5, 5, -5, 5]);
    const so2Space = new oxmpl.base.SO2StateSpace();
    const spaceBuilder = new oxmpl.base.CompoundStateSpaceBuilder();
    spaceBuilder.addRealVectorStateSpace(r2Space, 1.0);
    spaceBuilder.addSO2StateSpace(so2Space, 0.5);
    const space = spaceBuilder.build();

    const startBuilder = new oxmpl.base.CompoundStateBuilder();
    startBuilder.addRealVectorState(new oxmpl.base.RealVectorState([-2, 0]));
    startBuilder.addSO2State(new oxmpl.base.SO2State(0));
    const startState = startBuilder.build();

    const targetBuilder = new oxmpl.base.CompoundStateBuilder();
    targetBuilder.addRealVectorState(new oxmpl.base.RealVectorState([2, 0]));
    targetBuilder.addSO2State(new oxmpl.base.SO2State(Math.PI));
    const targetState = targetBuilder.build();

    const goalRegion = new CompoundGoal(space, targetState, 0.5);
    const goal = new oxmpl.base.Goal(goalRegion);

    const problemDef = oxmpl.base.ProblemDefinition.fromCompoundState(space, startState, goal);

    const validityCheckerFn = (state) => {
      const r2 = state.getComponent(0);
      const v = Array.from(r2.values);
      if (v[0] >= -0.5 && v[0] <= 0.5 && v[1] >= -2 && v[1] <= 2) return false;
      return true;
    };
    const validityChecker = new oxmpl.base.StateValidityChecker(validityCheckerFn);

    const planner_config = new oxmpl.base.PlannerConfig(0);
    const planner = new oxmpl.geometric.PRM(2.0, 1.5, problemDef, planner_config);
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
