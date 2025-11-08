import * as oxmpl from 'oxmpl';
import { describe, expect, test } from 'vitest';

class CircularGoal {
  constructor(space, x, y, radius) {
    this.space = space;
    this.target = [x, y];
    this.radius = radius;
    this.rng = Math.random;
  }

  isSatisfied(state) {
    const dx = state[0] - this.target[0];
    const dy = state[1] - this.target[1];
    const distance = Math.sqrt(dx * dx + dy * dy);
    return distance <= this.radius;
  }

  distanceToGoal(state) {
    const dx = state[0] - this.target[0];
    const dy = state[1] - this.target[1];
    const distance = Math.sqrt(dx * dx + dy * dy);
    return Math.max(0, distance - this.radius);
  }

  sampleGoal() {
    const angle = this.rng() * 2 * Math.PI;
    const radius = this.radius * Math.sqrt(this.rng());

    const x = this.target[0] + radius * Math.cos(angle);
    const y = this.target[1] + radius * Math.sin(angle);
    return new Float64Array([x, y]);
  }
}

function isStateValid(state) {
  const [x, y] = state;

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
    // DEFINE THE STATE SPACE
    // A 10x10 2D world
    const space = new oxmpl.RealVectorStateSpace(2, [0.0, 10.0, 0.0, 10.0]);

    // DEFINE THE PROBLEM
    const startState = [1.0, 5.0];
    const goalRegion = new CircularGoal(space, 9.0, 5.0, 0.5);

    const goal = new oxmpl.Goal(
      goalRegion.isSatisfied.bind(goalRegion),
      goalRegion.distanceToGoal.bind(goalRegion),
      goalRegion.sampleGoal.bind(goalRegion)
    );

    const problemDef = new oxmpl.ProblemDefinition(space, startState, goal);
    const validityChecker = new oxmpl.StateValidityChecker(isStateValid);

    // CREATE AND SETUP THE PLANNER
    const maxDistance = 0.5;
    const goalBias = 0.05;
    const planner_config = new oxmpl.PlannerConfig(0);
    const planner = new oxmpl.RRT(maxDistance, goalBias, planner_config);

    planner.setup(problemDef, validityChecker);

    // SOLVE THE PROBLEM
    console.log('\nAttempting to solve planning problem...');
    const timeoutSecs = 5.0;

    let path;
    try {
      path = planner.solve(timeoutSecs);
      console.log(`Solution found with ${path.length()} states.`);
    } catch (error) {
      throw new Error(`Planner failed to find a solution when one should exist. Error: ${error}`);
    }

    // VALIDATE THE SOLUTION PATH
    const states = path.getStates();
    const pathLength = path.length();

    expect(pathLength).toBeGreaterThan(1);
    expect(states.length).toBe(pathLength);

    // Check start position
    const pathStart = states[0];
    const startDistance = space.distance(pathStart, startState);
    expect(startDistance).toBeLessThan(1e-9);

    // Check goal is reached
    const pathEnd = states[states.length - 1];
    expect(goalRegion.isSatisfied(pathEnd)).toBe(true);

    // Validate all states in path are valid
    for (let i = 0; i < states.length; i++) {
      const state = states[i];
      expect(isStateValid(state)).toBe(true);
    }

    console.log('Path validation successful!');
  });
});
