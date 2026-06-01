// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::state::{RealVectorState, SO2State, State};

/// A fixed named state representing a planar rigid-body configuration in SE(2).
///
/// An SE(2) state consists of a planar translation `(x, y)` and a planar rotation `(yaw)`.
/// Construction is kept behind the public constructor so the state stays internally consistent and
/// yaw normalization remains observable through the public getters.
#[derive(Clone, Debug)]
pub struct SE2State {
    translation: RealVectorState,
    rotation: SO2State,
}

impl State for SE2State {}

impl SE2State {
    /// Creates a new `SE2State` from x, y, and yaw components.
    ///
    /// # Examples
    ///
    /// ```
    /// use std::f64::consts::PI;
    /// use oxmpl::base::state::SE2State;
    ///
    /// let state1 = SE2State::new(1.0, 2.0, PI / 2.0);
    /// assert_eq!(state1.get_x(), 1.0);
    /// assert_eq!(state1.get_y(), 2.0);
    /// assert!((state1.get_yaw() - (PI / 2.0)).abs() < 1e-9);
    ///
    /// let state2 = SE2State::new(3.0, 4.0, 3.0 * PI); // equivalent to PI
    /// assert!((state2.get_yaw() + PI).abs() < 1e-9);
    /// ```
    pub fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self::from_parts(RealVectorState::new(vec![x, y]), SO2State::new(yaw))
    }

    pub(crate) fn from_parts(translation: RealVectorState, rotation: SO2State) -> Self {
        assert_eq!(
            translation.values.len(),
            2,
            "SE2State translation must have exactly two components."
        );

        Self {
            translation,
            rotation: SO2State::new(rotation.value),
        }
    }

    pub(crate) fn translation_mut(&mut self) -> &mut RealVectorState {
        &mut self.translation
    }

    pub(crate) fn rotation_mut(&mut self) -> &mut SO2State {
        &mut self.rotation
    }

    /// Returns a reference to the translational component `(x, y)` of the state.
    pub fn get_translation(&self) -> &RealVectorState {
        &self.translation
    }

    /// Returns a reference to the rotational component `(yaw)` of the state.
    pub fn get_rotation(&self) -> &SO2State {
        &self.rotation
    }

    /// Returns the x-coordinate of the state.
    pub fn get_x(&self) -> f64 {
        self.translation.values[0]
    }

    /// Returns the y-coordinate of the state.
    pub fn get_y(&self) -> f64 {
        self.translation.values[1]
    }

    /// Returns the yaw (rotation) of the state in radians.
    pub fn get_yaw(&self) -> f64 {
        self.rotation.value
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::base::state::{RealVectorState, SO2State};
    use std::f64::consts::PI;

    #[test]
    fn test_se2_state_creation_and_getters() {
        let state = SE2State::new(1.5, -2.5, PI / 2.0);

        assert_eq!(state.get_x(), 1.5);
        assert_eq!(state.get_y(), -2.5);
        assert_eq!(state.get_yaw(), PI / 2.0);

        assert_eq!(
            state.get_translation(),
            &RealVectorState::new(vec![1.5, -2.5])
        );
        assert_eq!(state.get_rotation(), &SO2State::new(PI / 2.0));
    }

    #[test]
    fn test_se2_state_yaw_normalization() {
        let state1 = SE2State::new(1.0, 2.0, 3.0 * PI / 2.0);
        assert!((state1.get_yaw() - (-PI / 2.0)).abs() < 1e-9);

        let state2 = SE2State::new(1.0, 2.0, 5.0 * PI);
        assert!((state2.get_yaw() + PI).abs() < 1e-9);

        let state3 = SE2State::new(1.0, 2.0, -7.0 * PI / 2.0);
        assert!((state3.get_yaw() - (PI / 2.0)).abs() < 1e-9);
    }

    #[test]
    fn test_se2_state_clone() {
        let state1 = SE2State::new(10.0, 20.0, PI / 4.0);
        let state2 = state1.clone();

        assert_eq!(state1.get_x(), state2.get_x());
        assert_eq!(state1.get_y(), state2.get_y());
        assert_eq!(state1.get_yaw(), state2.get_yaw());

        assert_eq!(state1.get_translation(), state2.get_translation());
        assert_eq!(state1.get_rotation(), state2.get_rotation());
    }
}
