// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::state::State;
use std::f64::consts::PI;

/// A state representing a 2D rotation, an element of the Special Orthogonal group SO(2).
///
/// The angle is stored in radians and is canonically represented in the range `[-PI, PI)`.
#[derive(Clone, Debug, PartialEq)]
pub struct SO2State {
    /// Angle of rotation in radians.
    pub value: f64,
}
impl SO2State {
    /// Creates a new `SO2State`, normalising the input angle to range `[-PI, PI)`.
    ///
    /// This constructor ensures that no matter the input value, resulting state's value will be a
    /// valid, equivalent angle within `[-PI, PI)`.
    ///
    /// # Examples
    ///
    /// ```
    /// use std::f64::consts::PI;
    /// use oxmpl::base::state::SO2State;
    ///
    /// let state1 = SO2State::new(3.0 * PI / 2.0);
    /// assert!((state1.value - (-PI / 2.0)).abs() < 1e-9);
    ///
    /// let state2 = SO2State::new(5.0 * PI);
    /// assert!((state2.value + PI).abs() < 1e-9);
    /// ```
    pub fn new(val: f64) -> Self {
        SO2State {
            value: (val + PI).rem_euclid(2.0 * PI) - PI,
        }
    }

    /// Normalises the state's angle in-place to range `[-PI, PI)`.
    ///
    /// Method modifies the current state.
    ///
    /// # Examples
    ///
    /// ```
    /// use std::f64::consts::PI;
    /// use oxmpl::base::state::SO2State;
    ///
    /// let mut state = SO2State { value: 3.0 * PI / 2.0 }; // Equivalent to -PI/2
    /// state = state.normalise();
    ///
    /// assert!((state.value + PI / 2.0).abs() < 1e-9);
    /// ```
    pub fn normalise(&mut self) -> Self {
        SO2State {
            value: (self.value + PI).rem_euclid(2.0 * PI) - PI,
        }
    }
}
impl State for SO2State {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_so2_state_creation() {
        let state = SO2State { value: 1.0 };
        assert_eq!(state.value, 1.0);
    }

    #[test]
    fn test_so2_state_clone() {
        let state1 = SO2State { value: 1.0 };
        let state2 = state1.clone();
        assert_eq!(state1, state2);
    }

    #[test]
    fn test_so2_state_normalise() {
        let mut state1 = SO2State {
            value: 3.0 * PI / 2.0,
        };
        let state2 = state1.normalise();
        assert_eq!(state2.value, -PI / 2.0);
    }
}
