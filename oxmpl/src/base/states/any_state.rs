// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use std::{any::Any, fmt::Debug};

/// Public dynamic state abstraction for runtime-composed compound states.
///
/// `AnyState` is the advanced API companion to the core typed [`State`] trait. Use `AnyState` when a
/// state must be stored or passed through runtime composition, such as heterogeneous components in
/// a [`crate::base::state::CompoundState`].
///
/// ```
/// use oxmpl::base::state::{AnyState, RealVectorState};
///
/// let state: Box<dyn AnyState> = Box::new(RealVectorState::new(vec![1.0, 2.0]));
///
/// assert!(state.as_any().downcast_ref::<RealVectorState>().is_some());
/// ```
pub trait DynCloneAnyState {
    fn clone_box(&self) -> Box<dyn AnyState>;
}

// Provides `Box<dyn AnyState>` cloneability: concrete types satisfy `Clone`, and trait objects
// regain cloneability through `clone_box`.
impl<T> DynCloneAnyState for T
where
    T: AnyState + Clone + 'static,
{
    fn clone_box(&self) -> Box<dyn AnyState> {
        Box::new(self.clone())
    }
}

/// A dynamic state trait for runtime-composed state APIs.
pub trait AnyState: DynCloneAnyState + Debug + Any + Send + Sync + 'static {
    fn as_any(&self) -> &dyn Any;
}

/// Explicit `AnyState` implementations are provided for each type that participates in
/// compound-state composition. See `RealVectorState`, `SO2State`, `SO3State`, and
/// `CompoundState` for the concrete impls.
impl Clone for Box<dyn AnyState> {
    fn clone(&self) -> Self {
        self.clone_box()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::base::state::{
        CompoundState, RealVectorState, SO2State, SO3State,
    };
    use std::f64::consts::PI;

    fn assert_boxed_any_state_clone_and_downcast<T: 'static>(state: Box<dyn AnyState>) {
        let cloned = state.clone();
        assert!(state.as_any().downcast_ref::<T>().is_some());
        assert!(cloned.as_any().downcast_ref::<T>().is_some());
    }

    #[test]
    fn test_real_vector_state_any_state() {
        let state: Box<dyn AnyState> = Box::new(RealVectorState::new(vec![1.0, 2.0]));
        let cloned = state.clone();
        assert_eq!(
            state.as_any().downcast_ref::<RealVectorState>(),
            cloned.as_any().downcast_ref::<RealVectorState>()
        );
    }

    #[test]
    fn test_so2_state_any_state() {
        let state: Box<dyn AnyState> = Box::new(SO2State::new(PI / 2.0));
        assert!(state.as_any().downcast_ref::<SO2State>().is_some());
        assert_boxed_any_state_clone_and_downcast::<SO2State>(state);
    }

    #[test]
    fn test_so3_state_any_state() {
        let state: Box<dyn AnyState> = Box::new(SO3State::new(1.0, 2.0, 3.0, 4.0));
        assert!(state.as_any().downcast_ref::<SO3State>().is_some());
        assert_boxed_any_state_clone_and_downcast::<SO3State>(state);
    }

    #[test]
    fn test_compound_state_any_state() {
        let components: Vec<Box<dyn AnyState>> = vec![
            Box::new(RealVectorState::new(vec![1.0, 2.0])),
            Box::new(SO2State::new(PI)),
        ];
        let state: Box<dyn AnyState> = Box::new(CompoundState::new(components));
        assert!(state.as_any().downcast_ref::<CompoundState>().is_some());
        assert_boxed_any_state_clone_and_downcast::<CompoundState>(state);
    }

    #[test]
    fn test_compound_state_heterogeneous_components() {
        let components: Vec<Box<dyn AnyState>> = vec![
            Box::new(RealVectorState::new(vec![1.0, 2.0])),
            Box::new(SO2State::new(PI / 2.0)),
        ];
        let compound = CompoundState::new(components);
        let cloned = compound.clone();

        let c1 = &compound.components[0];
        let c2 = &cloned.components[0];
        assert!(c1.as_any().downcast_ref::<RealVectorState>().is_some());
        assert!(c2.as_any().downcast_ref::<RealVectorState>().is_some());

        let c1 = &compound.components[1];
        let c2 = &cloned.components[1];
        assert!(c1.as_any().downcast_ref::<SO2State>().is_some());
        assert!(c2.as_any().downcast_ref::<SO2State>().is_some());
    }
}
