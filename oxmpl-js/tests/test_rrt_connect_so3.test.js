import oxmpl from 'oxmpl-js';
import { describe, expect, test } from 'vitest';

function quaternionFromAxisAngle(axis, angle) {
  const halfAngle = angle * 0.5;
  const s = Math.sin(halfAngle);
  const c = Math.cos(halfAngle);
  const norm = Math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2);
  if (norm < 1e-9) return oxmpl.base.SO3State.identity();
  const ax = axis[0] / norm;
  const ay = axis[1] / norm;
  const az = axis[2] / norm;
  return new oxmpl.base.SO3State(ax * s, ay * s, az * s, c);
}

class SO3Goal {
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
    return new oxmpl.base.SO3State(this.target.x, this.target.y, this.target.z, this.target.w);
  }
}

describe('RRT-Connect SO3 Integration Tests', () => {
  test('RRT-Connect problem in SO3', () => {
    const space = new oxmpl.base.SO3StateSpace();
    const startState = quaternionFromAxisAngle([0, 1, 0], Math.PI / 2.0);
    const goalState = quaternionFromAxisAngle([0, 1, 0], -Math.PI / 2.0);
    const goalRegion = new SO3Goal(space, goalState, 0.2);
    const goal = new oxmpl.base.Goal(goalRegion);
    const problemDef = oxmpl.base.ProblemDefinition.fromSO3State(space, startState, goal);

    const identity = oxmpl.base.SO3State.identity();
    const validityCheckerFn = (state) => {
      return space.distance(identity, state) > 0.8;
    };
    const validityChecker = new oxmpl.base.StateValidityChecker(validityCheckerFn);

    const planner_config = new oxmpl.base.PlannerConfig(0);
    const planner = new oxmpl.geometric.RRTConnect(0.5, 0.1, problemDef, planner_config);
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
