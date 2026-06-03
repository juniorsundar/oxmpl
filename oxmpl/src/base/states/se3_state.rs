// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::state::{RealVectorState, SO3State, State};

/// A state representing a 3D rigid body transformation, an element of the Special Euclidean group
/// SE(3).
///
/// This state is composed of a 3D translation (x, y, z) and a 3D rotation.
#[derive(Clone, Debug)]
pub struct SE3State {
    translation: RealVectorState,
    rotation: SO3State,
}

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
        Self::from_parts(RealVectorState::new(vec![x, y, z]), rotation)
    }

    pub(crate) fn from_parts(translation: RealVectorState, mut rotation: SO3State) -> Self {
        assert_eq!(
            translation.values.len(),
            3,
            "SE3State translation must have exactly three components."
        );
        // Normalise rotation, falling back to identity for degenerate quaternions.
        rotation = rotation.normalise().unwrap_or(SO3State::identity());
        SE3State {
            translation,
            rotation,
        }
    }

    pub(crate) fn translation_mut(&mut self) -> &mut RealVectorState {
        &mut self.translation
    }

    pub(crate) fn rotation_mut(&mut self) -> &mut SO3State {
        &mut self.rotation
    }

    /// Returns a reference to the translational component (x, y, z) of the state.
    pub fn get_translation(&self) -> &RealVectorState {
        &self.translation
    }

    /// Returns a reference to the rotational component (`SO3State`) of the state.
    pub fn get_rotation(&self) -> &SO3State {
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

    /// Returns the z-coordinate of the state.
    pub fn get_z(&self) -> f64 {
        self.translation.values[2]
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

    #[test]
    fn test_se3_state_non_identity_rotation_observable() {
        // Non-identity unit quaternion: 90° rotation around Z axis
        // q = (0, 0, sin(π/4), cos(π/4)) = (0, 0, 0.707..., 0.707...)
        let rot = SO3State::new(
            0.0,
            0.0,
            std::f64::consts::FRAC_1_SQRT_2,
            std::f64::consts::FRAC_1_SQRT_2,
        );
        let state = SE3State::new(-3.0, 7.5, 1.25, rot.clone());

        assert_eq!(state.get_x(), -3.0);
        assert_eq!(state.get_y(), 7.5);
        assert_eq!(state.get_z(), 1.25);
        assert_eq!(state.get_rotation(), &rot);

        // Verify SO(3) quaternion semantics through get_rotation()
        let q = state.get_rotation();
        let norm = (q.x.powi(2) + q.y.powi(2) + q.z.powi(2) + q.w.powi(2)).sqrt();
        assert!(
            (norm - 1.0).abs() < 1e-9,
            "rotation quaternion should be unit"
        );
        // Z-axis rotation: x=y=0, z=sin(θ/2), w=cos(θ/2), θ=π/2
        assert!((q.z - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-9);
        assert!((q.w - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-9);
    }
}
