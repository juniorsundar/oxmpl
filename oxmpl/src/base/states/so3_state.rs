// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::{error::StateError, state::State};

/// A state representing a 3D rotation, an element of the Special Orthogonal group SO(3).
///
/// The rotation is stored as a unit quaternion `(x, y, z, w)`.
#[derive(Clone, Debug, PartialEq, Default)]
pub struct SO3State {
    /// The x-component of the quaternion's vector.
    pub x: f64,
    /// The y-component of the quaternion's vector.
    pub y: f64,
    /// The z-component of the quaternion's vector.
    pub z: f64,
    /// The w-component of the quaternion's scalar.
    pub w: f64,
}
impl SO3State {
    /// Creates a new `SO3State` with the given components.
    ///
    /// > [!NOTE]
    /// > This function does not check if the resulting quaternion is normalized. It is the user's
    /// > responsibility to provide components that form a unit quaternion or to call `normalise()`
    /// > afterward.
    pub fn new(x: f64, y: f64, z: f64, w: f64) -> Self {
        SO3State { x, y, z, w }
    }

    /// Normalises the state's quaternion, returning a new `SO3State` with magnitude 1.
    ///
    /// This method consumes the original state and returns a `Result`. On success, it
    /// returns `Ok(SO3State)` with the normalised quaternion. If the original state has
    /// zero magnitude, it returns `Err(StateError::ZeroMagnitude)`.
    ///
    /// # Examples
    ///
    /// ```
    /// use oxmpl::base::state::SO3State;
    ///
    /// let mut unnorm_state = SO3State::new(1.0, 1.0, 1.0, 1.0);
    /// let norm_state = unnorm_state.normalise().unwrap();
    ///
    /// // The magnitude of the new state is 1.0
    /// let mag = (norm_state.x.powi(2) + norm_state.y.powi(2) + norm_state.z.powi(2) + norm_state.w.powi(2)).sqrt();
    /// assert!((mag - 1.0).abs() < 1e-9);
    /// ```
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

    /// Returns the identity quaternion, representing no rotation.
    ///
    /// # Examples
    ///
    /// ```
    /// use oxmpl::base::state::SO3State;
    ///
    /// let identity = SO3State::identity();
    /// assert_eq!(identity, SO3State { x: 0.0, y: 0.0, z: 0.0, w: 1.0 });
    /// ```
    pub fn identity() -> Self {
        Self {
            x: 0.,
            y: 0.,
            z: 0.,
            w: 1.,
        }
    }
}
impl State for SO3State {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_so3_state_new() {
        let state = SO3State::new(1.0, 2.0, 3.0, 4.0);
        assert_eq!(state.x, 1.0);
        assert_eq!(state.y, 2.0);
        assert_eq!(state.z, 3.0);
        assert_eq!(state.w, 4.0);
    }

    #[test]
    fn test_so3_state_clone() {
        let state1 = SO3State::new(1.0, 2.0, 3.0, 4.0);
        let state2 = state1.clone();
        assert_eq!(state1, state2);
    }

    #[test]
    fn test_so3_state_identity() {
        let identity = SO3State::identity();
        assert_eq!(identity.x, 0.0);
        assert_eq!(identity.y, 0.0);
        assert_eq!(identity.z, 0.0);
        assert_eq!(identity.w, 1.0);
    }

    #[test]
    fn test_so3_state_normalise_ok() {
        // A quaternion with norm = sqrt(1+4+9+16) = sqrt(30)
        let mut state = SO3State::new(1.0, 2.0, 3.0, 4.0);
        let norm = 30.0f64.sqrt();

        let normalised_state = state.normalise().unwrap();

        assert!((normalised_state.x - (1.0 / norm)).abs() < 1e-9);
        assert!((normalised_state.y - (2.0 / norm)).abs() < 1e-9);
        assert!((normalised_state.z - (3.0 / norm)).abs() < 1e-9);
        assert!((normalised_state.w - (4.0 / norm)).abs() < 1e-9);
    }

    #[test]
    fn test_so3_state_normalise_err_on_zero_magnitude() {
        let mut zero_state = SO3State::new(0.0, 0.0, 0.0, 0.0);
        let result = zero_state.normalise();

        assert!(result.is_err());
        match result.err().unwrap() {
            StateError::ZeroMagnitude => (),
        }
    }
}
