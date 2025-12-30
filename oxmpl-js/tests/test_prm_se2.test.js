import oxmpl from 'oxmpl-js';
import { describe, expect, test } from 'vitest';

class SE2Goal {
  constructor(space, x, y, radius) {
    this.space = space;
    this.target = [x, y];
    this.radius = radius;
    this.rng = Math.random;
  }

  isSatisfied(state) {
    const dx = state.x - this.target[0];
    const dy = state.y - this.target[1];
    const distance = Math.sqrt(dx * dx + dy * dy);
    return distance <= this.radius;
  }

  distanceGoal(state) {
    const dx = state.x - this.target[0];
    const dy = state.y - this.target[1];
    const distance = Math.sqrt(dx * dx + dy * dy);
    return Math.max(0, distance - this.radius);
  }

  sampleGoal() {
    const angle = this.rng() * 2 * Math.PI;
    const radius = this.radius * Math.sqrt(this.rng());
    const x = this.target[0] + radius * Math.cos(angle);
    const y = this.target[1] + radius * Math.sin(angle);
    const yaw = this.rng() * 2 * Math.PI - Math.PI;
    return new oxmpl.base.SE2State(x, y, yaw);
  }
}

function isStateValid(state) {
  const x = state.x;
  const y = state.y;

  const distSq = x * x + y * y;
  const minSafeDistSq = 1.0;

  return distSq >= minSafeDistSq;
}

describe('PRM SE2 Integration Tests', () => {
  test('PRM problem in SE2 with obstacle', () => {
    const bounds = [-10.0, 10.0, -10.0, 10.0, -Math.PI, Math.PI];
    const space = new oxmpl.base.SE2StateSpace(1.0, bounds);

    const startState = new oxmpl.base.SE2State(-5.0, 0.0, 0.0);
    const goalRegion = new SE2Goal(space, 5.0, 0.0, 0.5);
    const goal = new oxmpl.base.Goal(goalRegion);

    const problemDef = oxmpl.base.ProblemDefinition.fromSE2State(space, startState, goal);
    const validityChecker = new oxmpl.base.StateValidityChecker(isStateValid);

    const roadmapTimeout = 2.0;
    const connectionRadius = 3.0;
    const planner_config = new oxmpl.base.PlannerConfig(0);
    const planner = new oxmpl.geometric.PRM(
      roadmapTimeout,
      connectionRadius,
      problemDef,
      planner_config
    );

    planner.setup(validityChecker);

    console.log('\nConstructing PRM roadmap (SE2)...');
    planner.constructRoadmap();

    console.log('Solving (SE2)...');
    const path = planner.solve(5.0);
    console.log(`Solution found with ${path.getLength()} states.`);

    const states = path.getStates();
    expect(states.length).toBeGreaterThan(1);

    expect(space.distance(states[0], startState)).toBeLessThan(1e-9);

    expect(goalRegion.isSatisfied(states[states.length - 1])).toBe(true);

    for (const state of states) {
      expect(isStateValid(state)).toBe(true);
    }

    console.log('SE2 Path validation successful!');
  });
});
