// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use std::{any::Any, ops::Deref};

use crate::base::state::{CompoundState, RealVectorState, SO2State, State};

/// A state representing a 2D rigid body transformation, an element of the Special Euclidean group
/// SE(2).
///
/// This state is composed of a 2D translation (x, y) and a 2D rotation (yaw). It is internally
/// represented as a `CompoundState`.
#[derive(Clone, Debug)]
pub struct SE2State(pub CompoundState);

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
        SE2State(CompoundState {
            components: vec![
                Box::new(RealVectorState::new(vec![x, y])),
                Box::new(SO2State::new(yaw)),
            ],
        })
    }

    /// Returns a reference to the translational component (x, y) of the state.
    pub fn get_translation(&self) -> &RealVectorState {
        (self.0.components[0].deref() as &dyn Any)
            .downcast_ref::<RealVectorState>()
            .expect("Issue found in retreiving the translation vector.")
    }

    /// Returns a reference to the rotational component (yaw) of the state.
    pub fn get_rotation(&self) -> &SO2State {
        (self.0.components[1].deref() as &dyn Any)
            .downcast_ref::<SO2State>()
            .expect("Issue found in retreiving the rotation.")
    }

    /// Returns the x-coordinate of the state.
    pub fn get_x(&self) -> f64 {
        (self.0.components[0].deref() as &dyn Any)
            .downcast_ref::<RealVectorState>()
            .expect("Issue found in retreiving the translation vector.")
            .values[0]
    }

    /// Returns the y-coordinate of the state.
    pub fn get_y(&self) -> f64 {
        (self.0.components[0].deref() as &dyn Any)
            .downcast_ref::<RealVectorState>()
            .expect("Issue found in retreiving the translation vector.")
            .values[1]
    }

    /// Returns the yaw (rotation) of the state in radians.
    pub fn get_yaw(&self) -> f64 {
        (self.0.components[1].deref() as &dyn Any)
            .downcast_ref::<SO2State>()
            .expect("Issue found in retreiving the rotation.")
            .value
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
