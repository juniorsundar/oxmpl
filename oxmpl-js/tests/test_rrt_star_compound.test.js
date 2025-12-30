import oxmpl from 'oxmpl-js';
import { describe, expect, test } from 'vitest';

class CompoundGoal {
  constructor(space, target, radius) {
    this.space = space;
    this.target = target;
    this.radius = radius;
    this.rng = Math.random;
  }

  isSatisfied(state) {
    return this.space.distance(state, this.target) <= this.radius;
  }

  distanceGoal(state) {
    const dist = this.space.distance(state, this.target);
    return Math.max(0, dist - this.radius);
  }

  sampleGoal() {
    const targetR2 = this.target.getComponent(0);
    const targetValues = Array.from(targetR2.values);

    const rvState = new oxmpl.base.RealVectorState(targetValues);

    const targetSO2 = this.target.getComponent(1);
    const so2State = new oxmpl.base.SO2State(targetSO2.value);

    const builder = new oxmpl.base.CompoundStateBuilder();
    builder.addRealVectorState(rvState);
    builder.addSO2State(so2State);
    return builder.build();
  }
}

function isStateValid(state) {
  const r2 = state.getComponent(0);
  const vals = Array.from(r2.values);
  const x = vals[0];
  const y = vals[1];

  const inBox = x >= -1 && x <= 1 && y >= -1 && y <= 1;
  return !inBox;
}

describe('RRT* Compound Integration Tests', () => {
  test('RRT* problem in Compound Space (R2 + SO2)', () => {
    const r2Space = new oxmpl.base.RealVectorStateSpace(2, [-5, 5, -5, 5]);
    const so2Space = new oxmpl.base.SO2StateSpace();

    const spaceBuilder = new oxmpl.base.CompoundStateSpaceBuilder();
    spaceBuilder.addRealVectorStateSpace(r2Space, 1.0);
    spaceBuilder.addSO2StateSpace(so2Space, 0.5);
    const space = spaceBuilder.build();

    const startBuilder = new oxmpl.base.CompoundStateBuilder();
    startBuilder.addRealVectorState(new oxmpl.base.RealVectorState([-4, 0]));
    startBuilder.addSO2State(new oxmpl.base.SO2State(0));
    const startState = startBuilder.build();

    const targetBuilder = new oxmpl.base.CompoundStateBuilder();
    targetBuilder.addRealVectorState(new oxmpl.base.RealVectorState([4, 0]));
    targetBuilder.addSO2State(new oxmpl.base.SO2State(Math.PI));
    const targetState = targetBuilder.build();

    const goalRegion = new CompoundGoal(space, targetState, 0.5);
    const goal = new oxmpl.base.Goal(goalRegion);

    const problemDef = oxmpl.base.ProblemDefinition.fromCompoundState(space, startState, goal);
    const validityChecker = new oxmpl.base.StateValidityChecker(isStateValid);

    const planner_config = new oxmpl.base.PlannerConfig(0);
    const planner = new oxmpl.geometric.RRTStar(1.0, 0.1, 2.0, problemDef, planner_config);

    planner.setup(validityChecker);

    console.log('Solving (Compound R2+SO2)...');
    const path = planner.solve(5.0);
    console.log(`Solution found with ${path.getLength()} states.`);

    const states = path.getStates();
    expect(states.length).toBeGreaterThan(1);

    expect(space.distance(states[0], startState)).toBeLessThan(1e-9);
    expect(goalRegion.isSatisfied(states[states.length - 1])).toBe(true);

    for (const state of states) {
      expect(isStateValid(state)).toBe(true);
    }

    console.log('Compound Path validation successful!');
  });
});
