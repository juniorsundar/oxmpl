import oxmpl from 'oxmpl-js';
import { describe, expect, test } from 'vitest';

class AngleGoal {
  constructor(space, targetAngle, radius) {
    this.space = space;
    this.target = new oxmpl.base.SO2State(targetAngle);
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
    const offset = (this.rng() * 2 - 1) * this.radius;
    return new oxmpl.base.SO2State(this.target.value + offset);
  }
}

function isAngleValid(state) {
  const angle = state.value;
  return !(angle > -0.5 && angle < 0.5);
}

describe('RRT SO2 Integration Tests', () => {
  test('RRT problem in SO2 with forbidden zone', () => {
    const space = new oxmpl.base.SO2StateSpace();

    const startState = new oxmpl.base.SO2State(-Math.PI / 2.0);
    const goalRegion = new AngleGoal(space, Math.PI / 2.0, 0.2);
    const goal = new oxmpl.base.Goal(goalRegion);

    const problemDef = oxmpl.base.ProblemDefinition.fromSO2State(space, startState, goal);
    const validityChecker = new oxmpl.base.StateValidityChecker(isAngleValid);

    const maxDistance = 0.5;
    const goalBias = 0.1;
    const planner_config = new oxmpl.base.PlannerConfig(0);
    const planner = new oxmpl.geometric.RRT(maxDistance, goalBias, problemDef, planner_config);

    planner.setup(validityChecker);

    console.log('Solving (SO2)...');
    const path = planner.solve(5.0);
    console.log(`Solution found with ${path.getLength()} states.`);

    const states = path.getStates();
    expect(states.length).toBeGreaterThan(1);

    for (const state of states) {
      expect(isAngleValid(state)).toBe(true);
    }

    console.log('SO2 Path validation successful!');
  });
});
