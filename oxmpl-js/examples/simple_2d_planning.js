// Simple 2D motion planning example using oxmpl-js
// This example demonstrates planning a path around a circular obstacle

import console from 'node:console';
import oxmpl from 'oxmpl-js';

// A state is invalid if it's inside a circular obstacle at the origin
function isStateValid(state) {
  const [x, y] = state.values;
  return Math.sqrt(x * x + y * y) > 2.0;
}

// Create a 2D state space with bounds
const space = new oxmpl.base.RealVectorStateSpace(2, [-10.0, 10.0, -10.0, 10.0]);

// Define start state
const start = new oxmpl.base.RealVectorState([-5.0, -5.0]);

// Define circular goal region
const target = [5.0, 5.0];
const radius = 0.5;
const goal = new oxmpl.base.Goal({
  isSatisfied: (state) => {
    const [x, y] = state.values;
    const dist = Math.sqrt((x - target[0]) ** 2 + (y - target[1]) ** 2);
    return dist <= radius;
  },
  distanceGoal: (state) => {
    const [x, y] = state.values;
    const dist = Math.sqrt((x - target[0]) ** 2 + (y - target[1]) ** 2);
    return Math.max(0, dist - radius);
  },
  sampleGoal: () => new oxmpl.base.RealVectorState(target), // Sample goal center
});

// Create problem and run planner
const problem = oxmpl.base.ProblemDefinition.fromRealVectorState(space, start, goal);
const validityChecker = new oxmpl.base.StateValidityChecker(isStateValid);

const plannerConfig = new oxmpl.base.PlannerConfig(0);
const planner = new oxmpl.geometric.RRT(0.5, 0.05, problem, plannerConfig);
planner.setup(validityChecker);

try {
  const path = planner.solve(5.0);
  if (path && path.getLength() > 0) {
    console.log(`Solution found with ${path.getLength()} states!`);
  } else {
    console.log('No solution found');
  }
} catch (e) {
  console.log(`Planning failed: ${e}`);
}
