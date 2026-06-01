import oxmpl from 'oxmpl-js';
import { describe, expect, test } from 'vitest';

describe('Compound binding boundaries', () => {
  test('compound state builder does not expose fixed named state methods', () => {
    const builder = new oxmpl.base.CompoundStateBuilder();

    expect(builder.addSE2State).toBeUndefined();
    expect(builder.addSE3State).toBeUndefined();
  });

  test('compound state space builder does not expose fixed named space methods', () => {
    const builder = new oxmpl.base.CompoundStateSpaceBuilder();

    expect(builder.addSE2StateSpace).toBeUndefined();
    expect(builder.addSE3StateSpace).toBeUndefined();
  });
});
