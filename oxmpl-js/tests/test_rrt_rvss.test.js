import oxmpl from 'oxmpl-js';
import { describe, expect, test } from 'vitest';

class CircularGoal {
  constructor(space, x, y, radius) {
    this.space = space;
    this.target = [x, y];
    this.radius = radius;
    this.rng = Math.random;
  }

  isSatisfied(state) {
    const [sx, sy] = state.values;
    const dx = sx - this.target[0];
    const dy = sy - this.target[1];
    const distance = Math.sqrt(dx * dx + dy * dy);
    return distance <= this.radius;
  }

  distanceGoal(state) {
    const [sx, sy] = state.values;
    const dx = sx - this.target[0];
    const dy = sy - this.target[1];
    const distance = Math.sqrt(dx * dx + dy * dy);
    return Math.max(0, distance - this.radius);
  }

  sampleGoal() {
    const angle = this.rng() * 2 * Math.PI;
    const radius = this.radius * Math.sqrt(this.rng());

    const x = this.target[0] + radius * Math.cos(angle);
    const y = this.target[1] + radius * Math.sin(angle);
    return new oxmpl.base.RealVectorState([x, y]);
  }
}

function isStateValid(state) {
  const [x, y] = state.values;

  const wallXPos = 5.0;
  const wallYMin = 2.0;
  const wallYMax = 8.0;
  const wallThickness = 0.5;

  const isInWall =
    x >= wallXPos - wallThickness / 2.0 &&
    x <= wallXPos + wallThickness / 2.0 &&
    y >= wallYMin &&
    y <= wallYMax;

  return !isInWall;
}

describe('RRT Integration Tests', () => {
  test('RRT problem with wall', () => {
    const space = new oxmpl.base.RealVectorStateSpace(2, [0.0, 10.0, 0.0, 10.0]);

    const startState = new oxmpl.base.RealVectorState([1.0, 5.0]);
    const goalRegion = new CircularGoal(space, 9.0, 5.0, 0.5);

    const goal = new oxmpl.base.Goal(goalRegion);

    const problemDef = oxmpl.base.ProblemDefinition.fromRealVectorState(space, startState, goal);
    const validityChecker = new oxmpl.base.StateValidityChecker(isStateValid);

    const maxDistance = 0.5;
    const goalBias = 0.05;
    const planner_config = new oxmpl.base.PlannerConfig(0);
    const planner = new oxmpl.geometric.RRT(maxDistance, goalBias, problemDef, planner_config);

    planner.setup(validityChecker);

    console.log('\nAttempting to solve planning problem...');
    const timeoutSecs = 5.0;

    let path;
    try {
      path = planner.solve(timeoutSecs);
      console.log(`Solution found with ${path.getLength()} states.`);
    } catch (error) {
      throw new Error(`Planner failed to find a solution when one should exist. Error: ${error}`);
    }

    const states = path.getStates();
    const pathLength = path.getLength();

    expect(pathLength).toBeGreaterThan(1);
    expect(states.length).toBe(pathLength);

    const pathStart = states[0];
    const startDistance = space.distance(pathStart, startState);
    expect(startDistance).toBeLessThan(1e-9);

    const pathEnd = states[states.length - 1];
    expect(goalRegion.isSatisfied(pathEnd)).toBe(true);

    for (let i = 0; i < states.length; i++) {
      const state = states[i];
      expect(isStateValid(state)).toBe(true);
    }

    console.log('Path validation successful!');
  });
});
