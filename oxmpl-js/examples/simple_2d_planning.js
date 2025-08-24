// Simple 2D motion planning example using oxmpl-js
// This example demonstrates planning a path around a circular obstacle

import console from 'node:console';
import * as oxmpl from '../pkg-bundler/oxmpl_js.js';

// A state is invalid if it's inside a circular obstacle at the origin
function isStateValid(state) {
  const [x, y] = state;
  return Math.sqrt(x * x + y * y) > 2.0;
}

// Create a 2D state space with bounds
const space = new oxmpl.RealVectorStateSpace(2, [-10.0, 10.0, -10.0, 10.0]);

// Define start state
const start = new Float64Array([-5.0, -5.0]);

// Define circular goal region
const target = [5.0, 5.0];
const radius = 0.5;
const goal = new oxmpl.Goal(
  (state) => {
    const [x, y] = state;
    const dist = Math.sqrt((x - target[0]) ** 2 + (y - target[1]) ** 2);
    return dist <= radius;
  },
  (state) => {
    const [x, y] = state;
    const dist = Math.sqrt((x - target[0]) ** 2 + (y - target[1]) ** 2);
    return Math.max(0, dist - radius);
  },
  () => new Float64Array(target) // Sample goal center
);

// Create problem and run planner
const problem = new oxmpl.ProblemDefinition(space, start, goal);
const validityChecker = new oxmpl.StateValidityChecker(isStateValid);

const planner = new oxmpl.RRT(0.5, 0.05);
planner.setup(problem, validityChecker);

try {
  const path = planner.solve(5.0);
  if (path && path.length() > 0) {
    console.log(`Solution found with ${path.length()} states!`);
  } else {
    console.log('No solution found');
  }
} catch (e) {
  console.log(`Planning failed: ${e}`);
}
