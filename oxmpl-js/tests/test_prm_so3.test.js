import oxmpl from 'oxmpl-js';
import { describe, expect, test } from 'vitest';

function quaternionFromAxisAngle(axis, angle) {
  const halfAngle = angle * 0.5;
  const s = Math.sin(halfAngle);
  const c = Math.cos(halfAngle);
  const norm = Math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
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

describe('PRM SO3 Integration Tests', () => {
  test('PRM problem in SO3 with forbidden zone', () => {
    const space = new oxmpl.base.SO3StateSpace();

    const startState = quaternionFromAxisAngle([0, 1, 0], Math.PI / 2.0);
    const goalState = quaternionFromAxisAngle([0, 1, 0], -Math.PI / 2.0);

    const goalRadius = 20.0 * (Math.PI / 180.0);
    const goalRegion = new SO3Goal(space, goalState, goalRadius);
    const goal = new oxmpl.base.Goal(goalRegion);

    const problemDef = oxmpl.base.ProblemDefinition.fromSO3State(space, startState, goal);

    const cosThreshold = Math.cos((30.0 * Math.PI) / 180.0 / 2.0); // ~0.966

    const validityCheckerFn = (state) => {
      return Math.abs(state.w) < cosThreshold;
    };
    const validityChecker = new oxmpl.base.StateValidityChecker(validityCheckerFn);

    const roadmapTimeout = 3.0;
    const connectionRadius = 1.0;
    const planner_config = new oxmpl.base.PlannerConfig(0);
    const planner = new oxmpl.geometric.PRM(
      roadmapTimeout,
      connectionRadius,
      problemDef,
      planner_config
    );

    planner.setup(validityChecker);

    console.log('Constructing PRM roadmap (SO3)...');
    planner.constructRoadmap();

    console.log('Solving (SO3)...');
    const path = planner.solve(5.0);
    console.log(`Solution found with ${path.getLength()} states.`);

    const states = path.getStates();
    expect(states.length).toBeGreaterThan(1);
    expect(space.distance(states[0], startState)).toBeLessThan(1e-9);
    expect(goalRegion.isSatisfied(states[states.length - 1])).toBe(true);

    for (const state of states) {
      expect(validityCheckerFn(state)).toBe(true);
    }
    console.log('SO3 Path validation successful!');
  }, 20000);
});
