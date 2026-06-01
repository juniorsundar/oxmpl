import math
import pytest

from oxmpl_py.base import (
    CompoundState,
    CompoundStateSpace,
    RealVectorState,
    RealVectorStateSpace,
    SE2State,
    SE2StateSpace,
    SE3State,
    SE3StateSpace,
    SO2State,
    SO2StateSpace,
    SO3State,
)


def test_compound_state_rejects_fixed_named_se2_components():
    with pytest.raises(ValueError, match="SE2State"):
        CompoundState([RealVectorState([1.0, 2.0]), SE2State(0.0, 0.0, 0.0)])



def test_compound_state_space_rejects_fixed_named_se2_subspaces():
    with pytest.raises(ValueError, match="SE2StateSpace"):
        CompoundStateSpace(
            [RealVectorStateSpace(dimension=2, bounds=[(-1.0, 1.0), (-1.0, 1.0)]), SE2StateSpace(weight=1.0, bounds=[(-1.0, 1.0), (-1.0, 1.0), (-math.pi, math.pi)])],
            weights=[1.0, 0.5],
        )



def test_compound_state_rejects_fixed_named_se3_components():
    with pytest.raises(ValueError, match="SE3State"):
        CompoundState(
            [
                RealVectorState([1.0, 2.0]),
                SE3State(0.0, 0.0, 0.0, SO3State(0.0, 0.0, 0.0, 1.0)),
            ]
        )



def test_compound_state_space_rejects_fixed_named_se3_subspaces():
    with pytest.raises(ValueError, match="SE3StateSpace"):
        CompoundStateSpace(
            [
                RealVectorStateSpace(dimension=2, bounds=[(-1.0, 1.0), (-1.0, 1.0)]),
                SE3StateSpace(
                    weight=1.0,
                    bounds=[(-1.0, 1.0), (-1.0, 1.0), (-1.0, 1.0)],
                ),
            ],
            weights=[1.0, 0.5],
        )



def test_compound_state_and_space_still_accept_dynamic_components():
    state = CompoundState([RealVectorState([1.0, 2.0]), SO2State(0.0)])
    space = CompoundStateSpace(
        [
            RealVectorStateSpace(dimension=2, bounds=[(-1.0, 1.0), (-1.0, 1.0)]),
            SO2StateSpace(),
        ],
        weights=[1.0, 0.5],
    )

    assert len(state.components) == 2
    assert space.distance(state, state) == 0.0



def test_compound_state_and_space_accept_nested_compounds():
    inner_state = CompoundState([RealVectorState([0.25, -0.25]), SO2State(0.1)])
    outer_state = CompoundState([inner_state, SO2State(-0.2)])

    inner_space = CompoundStateSpace(
        [
            RealVectorStateSpace(dimension=2, bounds=[(-1.0, 1.0), (-1.0, 1.0)]),
            SO2StateSpace(),
        ],
        weights=[1.0, 0.5],
    )
    outer_space = CompoundStateSpace([inner_space, SO2StateSpace()], weights=[1.0, 0.25])

    assert len(outer_state.components) == 2
    assert len(outer_state.components[0].components) == 2
    assert outer_space.distance(outer_state, outer_state) == 0.0
