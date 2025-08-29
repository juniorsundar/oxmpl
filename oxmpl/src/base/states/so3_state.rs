// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::{error::StateError, state::State};

#[derive(Clone, Debug, PartialEq)]
pub struct SO3State {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}
impl SO3State {
    pub fn new(x: f64, y: f64, z: f64, w: f64) -> Self {
        SO3State { x, y, z, w }
    }

    pub fn normalise(&mut self) -> Result<Self, StateError> {
        let norm = (self.x.powi(2) + self.y.powi(2) + self.z.powi(2) + self.w.powi(2)).sqrt();
        if norm < 1e-9 {
            Err(StateError::ZeroMagnitude)
        } else {
            Ok(SO3State {
                x: self.x / norm,
                y: self.y / norm,
                z: self.z / norm,
                w: self.w / norm,
            })
        }
    }
}
impl State for SO3State {}

// #[cfg(test)]
// mod tests {
//     use super::*;
//
//     #[test]
//     fn test_so2_state_creation() {
//         let state = SO3State { value: 1.0 };
//         assert_eq!(state.value, 1.0);
//     }
//
//     #[test]
//     fn test_so2_state_clone() {
//         let state1 = SO3State { value: 1.0 };
//         let state2 = state1.clone();
//         assert_eq!(state1, state2);
//     }
//
//     #[test]
//     fn test_so2_state_normalise() {
//         let mut state1 = SO3State {
//             value: 3.0 * PI / 2.0,
//         };
//         let state2 = state1.normalise();
//         assert_eq!(state2.value, -PI / 2.0);
//     }
// }
//
