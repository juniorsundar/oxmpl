import oxmpl from 'oxmpl-js';

// Define the validity checker
// State is invalid if the position (x, y) is inside a box obstacle
function isStateValid(state) {
  try {
    const rvPart = state.getComponent(0);
    const [x, y] = rvPart.values;

    // Box obstacle
    const x_min = -0.5,
      x_max = 0.5;
    const y_min = -2.0,
      y_max = 2.0;

    const insideBox = x >= x_min && x <= x_max && y >= y_min && y <= y_max;

    return !insideBox;
  } catch (e) {
    console.error('Validity check error:', e);
    return false;
  }
}

// 1. Create State Space
const r2 = new oxmpl.base.RealVectorStateSpace(2, [-5.0, 5.0, -5.0, 5.0]);
const so2 = new oxmpl.base.SO2StateSpace();

const spaceBuilder = new oxmpl.base.CompoundStateSpaceBuilder();
spaceBuilder.addRealVectorStateSpace(r2, 1.0);
spaceBuilder.addSO2StateSpace(so2, 0.5);
const space = spaceBuilder.build();

// 2. Define Start State
const startBuilder = new oxmpl.base.CompoundStateBuilder();
startBuilder.addRealVectorState(new oxmpl.base.RealVectorState([-2.0, 0.0]));
startBuilder.addSO2State(new oxmpl.base.SO2State(0.0));
const startState = startBuilder.build();

// 3. Define Goal Region
const targetBuilder = new oxmpl.base.CompoundStateBuilder();
targetBuilder.addRealVectorState(new oxmpl.base.RealVectorState([2.0, 0.0]));
targetBuilder.addSO2State(new oxmpl.base.SO2State(Math.PI));
const targetState = targetBuilder.build();

const radius = 0.5;

const goal = new oxmpl.base.Goal({
  isSatisfied: (state) => {
    return space.distance(state, targetState) <= radius;
  },
  distanceGoal: (state) => {
    const dist = space.distance(state, targetState);
    return Math.max(0, dist - radius);
  },
  sampleGoal: () => {
    // Sample a random point within radius of the target position
    // For simplicity in this example, we sample a random angle and distance
    // But we need to construct a proper CompoundState

    const tRV = targetState.getComponent(0);
    const [tx, ty] = tRV.values;

    const angle = Math.random() * 2 * Math.PI;
    const r = radius * Math.sqrt(Math.random());

    const x = tx + r * Math.cos(angle);
    const y = ty + r * Math.sin(angle);

    // Random orientation for goal
    const theta = Math.random() * 2 * Math.PI;

    const builder = new oxmpl.base.CompoundStateBuilder();
    builder.addRealVectorState(new oxmpl.base.RealVectorState([x, y]));
    builder.addSO2State(new oxmpl.base.SO2State(theta));
    return builder.build();
  },
});

// 4. Create Problem
const problem = oxmpl.base.ProblemDefinition.fromCompoundState(space, startState, goal);
const validityChecker = new oxmpl.base.StateValidityChecker(isStateValid);
const plannerConfig = new oxmpl.base.PlannerConfig(123);

// 5. Setup Planner
const planner = new oxmpl.geometric.RRT(0.5, 0.1, problem, plannerConfig);
planner.setup(validityChecker);

// 6. Solve
console.log('Solving Compound State planning problem (JS)...');
try {
  const path = planner.solve(5.0);
  if (path && path.getLength() > 0) {
    const states = path.getStates();
    console.log(`Solution found with ${states.length} states!`);

    if (states.length > 0) {
      const start = states[0];
      const end = states[states.length - 1];

      const sRV = start.getComponent(0);
      const sSO2 = start.getComponent(1);
      const eRV = end.getComponent(0);
      const eSO2 = end.getComponent(1);

      console.log(
        `Start: (${sRV.values[0].toFixed(2)}, ${sRV.values[1].toFixed(2)}, ${sSO2.value.toFixed(2)})`
      );
      console.log(
        `End:   (${eRV.values[0].toFixed(2)}, ${eRV.values[1].toFixed(2)}, ${eSO2.value.toFixed(2)})`
      );
    }
  } else {
    console.log('No solution found');
  }
} catch (e) {
  console.log(`Planning failed: ${e}`);
}
