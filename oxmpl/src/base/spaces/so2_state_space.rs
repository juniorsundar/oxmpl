// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use rand::Rng;
use std::f64::consts::PI;

use crate::base::{error::StateSamplingError, space::StateSpace, state::SO2State};

pub struct SO2StateSpace {
    // pub bounds: Vec<(f64, f64)>,
}

impl StateSpace for SO2StateSpace {
    type StateType = SO2State;

    fn distance(&self, state1: &Self::StateType, state2: &Self::StateType) -> f64 {
        let mut diff = state1.value - state2.value;
        diff = (diff + PI).rem_euclid(2.0 * PI) - PI;
        diff.abs()
    }

    fn interpolate(
        &self,
        from: &Self::StateType,
        to: &Self::StateType,
        t: f64,
        out_state: &mut Self::StateType,
    ) {
        let mut diff_to_from = to.clone().normalise().value - from.clone().normalise().value;
        if diff_to_from > PI {
            diff_to_from -= 2.0 * PI;
        } else if diff_to_from < -PI {
            diff_to_from += 2.0 * PI;
        }
        out_state.value = from.value + diff_to_from * t;
        out_state.value = out_state.normalise().value;
    }

    // TODO
    fn enforce_bounds(&self, _state: &mut Self::StateType) {}

    // TODO
    fn satisfies_bounds(&self, _state: &Self::StateType) -> bool {
        false
    }

    fn sample_uniform(&self, _rng: &mut impl Rng) -> Result<SO2State, StateSamplingError> {
        todo!()
    }

    fn get_longest_valid_segment_length(&self) -> f64 {
        todo!()
    }
}
