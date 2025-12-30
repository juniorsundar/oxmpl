import oxmpl from 'oxmpl-js';
import { describe, expect, test } from 'vitest';

class SE3Goal {
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
    const r = this.radius * Math.random();
    const theta = Math.random() * 2 * Math.PI;
    const phi = Math.random() * Math.PI;

    const dx = r * Math.sin(phi) * Math.cos(theta);
    const dy = r * Math.sin(phi) * Math.sin(theta);
    const dz = r * Math.cos(phi);

    const x = this.target.x + dx;
    const y = this.target.y + dy;
    const z = this.target.z + dz;

    return new oxmpl.base.SE3State(x, y, z, oxmpl.base.SO3State.identity());
  }
}

function isStateValid(state) {
  const x = state.x;
  const y = state.y;
  const z = state.z;

  const distSq = x * x + y * y + z * z;
  const minSafeDistSq = 1.0;

  return distSq >= minSafeDistSq;
}

describe('RRT-Connect SE3 Integration Tests', () => {
  test('RRT-Connect problem in SE3 with obstacle', () => {
    const bounds = [-10, 10, -10, 10, -10, 10];
    const space = new oxmpl.base.SE3StateSpace(1.0, bounds);

    const startState = new oxmpl.base.SE3State(-5.0, 0.0, 0.0, oxmpl.base.SO3State.identity());
    const targetState = new oxmpl.base.SE3State(5.0, 0.0, 0.0, oxmpl.base.SO3State.identity());

    const goalRegion = new SE3Goal(space, targetState, 0.5);
    const goal = new oxmpl.base.Goal(goalRegion);

    const problemDef = oxmpl.base.ProblemDefinition.fromSE3State(space, startState, goal);
    const validityChecker = new oxmpl.base.StateValidityChecker(isStateValid);

    const planner_config = new oxmpl.base.PlannerConfig(0);
    const planner = new oxmpl.geometric.RRTConnect(1.0, 0.1, problemDef, planner_config);

    planner.setup(validityChecker);

    console.log('Solving (SE3)...');
    const path = planner.solve(5.0);
    console.log(`Solution found with ${path.getLength()} states.`);

    const states = path.getStates();
    expect(states.length).toBeGreaterThan(1);

    expect(space.distance(states[0], startState)).toBeLessThan(1e-9);
    expect(goalRegion.isSatisfied(states[states.length - 1])).toBe(true);

    for (const state of states) {
      expect(isStateValid(state)).toBe(true);
    }

    console.log('SE3 Path validation successful!');
  });
});
