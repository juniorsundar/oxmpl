// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use std::{any::Any, ops::Deref};

use crate::base::state::{CompoundState, RealVectorState, SO3State, State};

/// A state representing a 3D rigid body transformation, an element of the Special Euclidean group
/// SE(3).
///
/// This state is composed of a 3D translation (x, y, z) and a 3D rotation. It is internally
/// represented as a `CompoundState`.
#[derive(Clone)]
pub struct SE3State(pub CompoundState);

impl State for SE3State {}

impl SE3State {
    /// Creates a new `SE3State` from x, y, z, and an `SO3State` for rotation.
    ///
    /// # Examples
    ///
    /// ```
    /// use oxmpl::base::state::{SE3State, SO3State};
    ///
    /// // Assuming SO3State can be created, for example, from a quaternion
    /// let rotation = SO3State::identity(); // Example identity rotation
    /// let state = SE3State::new(1.0, 2.0, 3.0, rotation);
    ///
    /// assert_eq!(state.get_x(), 1.0);
    /// assert_eq!(state.get_y(), 2.0);
    /// assert_eq!(state.get_z(), 3.0);
    /// ```
    pub fn new(x: f64, y: f64, z: f64, rotation: SO3State) -> Self {
        SE3State(CompoundState {
            components: vec![
                Box::new(RealVectorState::new(vec![x, y, z])),
                Box::new(rotation),
            ],
        })
    }

    /// Returns a reference to the translational component (x, y, z) of the state.
    pub fn get_translation(&self) -> &RealVectorState {
        (self.0.components[0].deref() as &dyn Any)
            .downcast_ref::<RealVectorState>()
            .expect("Issue found in retreiving the translation vector.")
    }

    /// Returns a reference to the rotational component (`SO3State`) of the state.
    pub fn get_rotation(&self) -> &SO3State {
        (self.0.components[1].deref() as &dyn Any)
            .downcast_ref::<SO3State>()
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

    /// Returns the z-coordinate of the state.
    pub fn get_z(&self) -> f64 {
        (self.0.components[0].deref() as &dyn Any)
            .downcast_ref::<RealVectorState>()
            .expect("Issue found in retreiving the translation vector.")
            .values[2]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::base::state::{RealVectorState, SO3State};

    #[test]
    fn test_se3_state_creation_and_getters() {
        let rotation = SO3State::identity();
        let state = SE3State::new(1.5, -2.5, 3.5, rotation.clone());

        assert_eq!(state.get_x(), 1.5);
        assert_eq!(state.get_y(), -2.5);
        assert_eq!(state.get_z(), 3.5);

        assert_eq!(
            state.get_translation(),
            &RealVectorState::new(vec![1.5, -2.5, 3.5])
        );
        assert_eq!(state.get_rotation(), &rotation);
    }

    #[test]
    fn test_se3_state_clone() {
        let rotation = SO3State::identity();
        let state1 = SE3State::new(10.0, 20.0, 30.0, rotation);
        let state2 = state1.clone();

        assert_eq!(state1.get_x(), state2.get_x());
        assert_eq!(state1.get_y(), state2.get_y());
        assert_eq!(state1.get_z(), state2.get_z());

        assert_eq!(state1.get_translation(), state2.get_translation());
        assert_eq!(state1.get_rotation(), state2.get_rotation());
    }
}
