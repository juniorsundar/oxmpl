import oxmpl from 'oxmpl-js';
import { describe, expect, test } from 'vitest';

describe('CompoundState and CompoundStateSpace', () => {
  test('Creation and Component Access', () => {
    const rvState = new oxmpl.base.RealVectorState([1.0, 2.0]);
    const so2State = new oxmpl.base.SO2State(1.5);

    const builder = new oxmpl.base.CompoundStateBuilder();
    builder.addRealVectorState(rvState);
    builder.addSO2State(so2State);
    const compound = builder.build();

    expect(compound.componentCount).toBe(2);

    const c1 = compound.getComponent(0);
    expect(c1).toBeInstanceOf(oxmpl.base.RealVectorState);
    expect(c1.values).toBeDefined();
    expect(Array.from(c1.values)).toEqual([1.0, 2.0]);

    const c2 = compound.getComponent(1);
    expect(c2).toBeInstanceOf(oxmpl.base.SO2State);
    expect(c2.value).toBe(1.5);
  });

  test('CompoundStateSpace Operations', () => {
    const rvSpace = new oxmpl.base.RealVectorStateSpace(2, [0, 5, 0, 5]);
    const so2Space = new oxmpl.base.SO2StateSpace();

    const spaceBuilder = new oxmpl.base.CompoundStateSpaceBuilder();
    spaceBuilder.addRealVectorStateSpace(rvSpace, 1.0);
    spaceBuilder.addSO2StateSpace(so2Space, 0.5);
    const compoundSpace = spaceBuilder.build();

    const builder1 = new oxmpl.base.CompoundStateBuilder();
    builder1.addRealVectorState(new oxmpl.base.RealVectorState([1.0, 1.0]));
    builder1.addSO2State(new oxmpl.base.SO2State(0.0));
    const state1 = builder1.build();

    const builder2 = new oxmpl.base.CompoundStateBuilder();
    builder2.addRealVectorState(new oxmpl.base.RealVectorState([4.0, 5.0]));
    builder2.addSO2State(new oxmpl.base.SO2State(Math.PI / 2.0));
    const state2 = builder2.build();

    const expectedDist = Math.sqrt(25 + Math.pow(Math.PI / 4.0, 2));
    const dist = compoundSpace.distance(state1, state2);
    expect(dist).toBeCloseTo(expectedDist, 4);

    const sampled = compoundSpace.sample();
    expect(sampled.componentCount).toBe(2);
    expect(compoundSpace.satisfiesBounds(sampled)).toBe(true);

    const interpolated = compoundSpace.interpolate(state1, state2, 0.5);
    const i1 = interpolated.getComponent(0); // RealVector
    const i2 = interpolated.getComponent(1); // SO2

    // Midpoint of [1,1] and [4,5] is [2.5, 3.0]
    expect(i1.values[0]).toBeCloseTo(2.5);
    expect(i1.values[1]).toBeCloseTo(3.0);

    // Midpoint of 0 and PI/2 is PI/4
    expect(i2.value).toBeCloseTo(Math.PI / 4.0);
  });
});
