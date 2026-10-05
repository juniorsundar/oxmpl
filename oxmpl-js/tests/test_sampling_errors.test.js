import oxmpl from 'oxmpl-js';
import { describe, expect, test } from 'vitest';

function unboundedProblem() {
  const space = new oxmpl.base.RealVectorStateSpace(1);
  const start = new oxmpl.base.RealVectorState([0.0]);
  const goal = new oxmpl.base.Goal({
    isSatisfied(state) {
      return state.values[0] === 1.0;
    },
    distanceGoal(state) {
      return Math.abs(state.values[0] - 1.0);
    },
    sampleGoal() {
      // RRTConnect needs a successful goal-root sample before uniform sampling.
      return new oxmpl.base.RealVectorState([1.0]);
    },
  });
  return oxmpl.base.ProblemDefinition.fromRealVectorState(space, start, goal);
}

describe('Planner sampling errors', () => {
  test('PRM surfaces sampling error', () => {
    const planner = new oxmpl.geometric.PRM(
      1.0,
      0.5,
      unboundedProblem(),
      new oxmpl.base.PlannerConfig(42)
    );
    planner.setup(new oxmpl.base.StateValidityChecker(() => true));

    expect(() => planner.constructRoadmap()).toThrow(
      expect.stringMatching(/Sampling.*dimension 0 is unbounded/)
    );
  });

  test('RRT surfaces sampling error', () => {
    const planner = new oxmpl.geometric.RRT(
      0.5,
      0.0,
      unboundedProblem(),
      new oxmpl.base.PlannerConfig(42)
    );
    planner.setup(new oxmpl.base.StateValidityChecker(() => true));

    expect(() => planner.solve(1.0)).toThrow(
      expect.stringMatching(/Sampling.*dimension 0 is unbounded/)
    );
  });

  test('RRTStar surfaces sampling error', () => {
    const planner = new oxmpl.geometric.RRTStar(
      0.5,
      0.0,
      0.25,
      unboundedProblem(),
      new oxmpl.base.PlannerConfig(42)
    );
    planner.setup(new oxmpl.base.StateValidityChecker(() => true));

    expect(() => planner.solve(1.0)).toThrow(
      expect.stringMatching(/Sampling.*dimension 0 is unbounded/)
    );
  });

  test('RRTConnect surfaces sampling error', () => {
    const planner = new oxmpl.geometric.RRTConnect(
      0.5,
      0.0,
      unboundedProblem(),
      new oxmpl.base.PlannerConfig(42)
    );
    planner.setup(new oxmpl.base.StateValidityChecker(() => true));

    expect(() => planner.solve(1.0)).toThrow(
      expect.stringMatching(/Sampling.*dimension 0 is unbounded/)
    );
  });
});
