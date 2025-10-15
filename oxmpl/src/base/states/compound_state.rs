// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::state::State;

/// A state that is composed of one or more other states.
///
/// This is useful for representing complex states, such as the state of a robot with multiple
/// joints, or a rigid body in space (which is composed of a translation and a rotation).
#[derive(Clone, Debug)]
pub struct CompoundState {
    /// The individual states that form this compound state.
    pub components: Vec<Box<dyn State>>,
}

impl CompoundState {
    /// Creates a new `CompoundState`.
    pub fn new(components: Vec<Box<dyn State>>) -> Self {
        CompoundState { components }
    }
}

impl State for CompoundState {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::base::state::RealVectorState;
    use std::any::Any;
    use std::ops::Deref;

    #[test]
    fn test_compound_state_creation() {
        let state1 = RealVectorState::new(vec![1.0, 2.0]);
        let state2 = RealVectorState::new(vec![3.0, 4.0]);
        let compound_state = CompoundState::new(vec![Box::new(state1), Box::new(state2)]);
        assert_eq!(compound_state.components.len(), 2);
    }

    #[test]
    fn test_compound_state_clone() {
        let state1 = RealVectorState::new(vec![1.0, 2.0]);
        let state2 = RealVectorState::new(vec![3.0, 4.0]);
        let compound_state1 =
            CompoundState::new(vec![Box::new(state1.clone()), Box::new(state2.clone())]);
        let compound_state2 = compound_state1.clone();
        assert_eq!(
            compound_state1.components.len(),
            compound_state2.components.len()
        );

        let s1_c1 = (compound_state1.components[0].deref() as &dyn Any)
            .downcast_ref::<RealVectorState>()
            .unwrap();
        let s2_c1 = (compound_state2.components[0].deref() as &dyn Any)
            .downcast_ref::<RealVectorState>()
            .unwrap();

        assert_eq!(s1_c1, s2_c1);

        let s1_c2 = (compound_state1.components[1].deref() as &dyn Any)
            .downcast_ref::<RealVectorState>()
            .unwrap();
        let s2_c2 = (compound_state2.components[1].deref() as &dyn Any)
            .downcast_ref::<RealVectorState>()
            .unwrap();
        assert_eq!(s1_c2, s2_c2);
    }
}
