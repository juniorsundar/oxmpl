// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use rand::RngCore;
use std::{any::Any, clone::Clone};

use crate::base::{
    error::StateSamplingError,
    space::StateSpace,
    state::{AnyState, State},
};

pub trait DynCloneAnyStateSpace {
    fn clone_box(&self) -> Box<dyn AnyStateSpace>;
}

impl<T> DynCloneAnyStateSpace for T
where
    T: AnyStateSpace + Clone + 'static,
{
    fn clone_box(&self) -> Box<dyn AnyStateSpace> {
        Box::new(self.clone())
    }
}

impl Clone for Box<dyn AnyStateSpace> {
    fn clone(&self) -> Self {
        self.clone_box()
    }
}

/// A helper trait for dynamic dispatch on `StateSpace` objects.
///
/// This trait enables "type erasure" for state spaces. It provides a way to call the methods of a
/// `StateSpace` without knowing its concrete type at compile time. This is the key mechanism that
/// allows `CompoundStateSpace` to hold a collection of different state space types in a single
/// `Vec`.
///
/// Each method in this trait is a dynamically-dispatchable version of the corresponding method in
/// the `StateSpace` trait, designed to work with trait objects like `&dyn AnyState` and `&mut dyn
/// AnyState`.
///
/// This is an internal implementation detail and is not typically used directly by end-users.
pub trait AnyStateSpace: DynCloneAnyStateSpace {
    /// A dynamically-dispatchable version of `StateSpace::distance`.
    ///
    /// # Panics
    /// Panics if the concrete types of `state1` or `state2` do not match the `StateType`
    /// associated with the underlying concrete `StateSpace`.
    fn distance_dyn(&self, state1: &dyn AnyState, state2: &dyn AnyState) -> f64;

    /// A dynamically-dispatchable version of `StateSpace::interpolate`.
    ///
    /// # Panics
    /// Panics if the concrete types of the states do not match the `StateType` associated with the
    /// underlying concrete `StateSpace`.
    fn interpolate_dyn(
        &self,
        from: &dyn AnyState,
        to: &dyn AnyState,
        t: f64,
        state: &mut dyn AnyState,
    );

    /// A dynamically-dispatchable version of `StateSpace::enforce_bounds`.
    ///
    /// # Panics
    /// Panics if the concrete type of `state` does not match the `StateType` associated with the
    /// underlying concrete `StateSpace`.
    fn enforce_bounds_dyn(&self, state: &mut dyn AnyState);

    /// A dynamically-dispatchable version of `StateSpace::satisfies_bounds`.
    ///
    /// # Panics
    /// Panics if the concrete type of `state` does not match the `StateType` associated with the
    /// underlying concrete `StateSpace`.
    fn satisfies_bounds_dyn(&self, state: &dyn AnyState) -> bool;

    /// A dynamically-dispatchable version of `StateSpace::sample_uniform`.
    ///
    /// Returns a `Box<dyn AnyState>` because the concrete state type is not known at compile time.
    fn sample_uniform_dyn(
        &self,
        rng: &mut dyn RngCore,
    ) -> Result<Box<dyn AnyState>, StateSamplingError>;

    /// A dynamically-dispatchable version of `StateSpace::get_longest_valid_segment_length`.
    fn get_longest_valid_segment_length_dyn(&self) -> f64;
}

/// Provides a blanket implementation of `AnyStateSpace` for any type that implements `StateSpace`.
/// It works by downcasting the generic `&dyn AnyState` trait objects back to their concrete types
/// at runtime.
impl<T: StateSpace + Clone + 'static> AnyStateSpace for T
where
    T::StateType: State + AnyState,
{
    fn distance_dyn(&self, state1: &dyn AnyState, state2: &dyn AnyState) -> f64 {
        let s1 = (state1 as &dyn Any).downcast_ref::<T::StateType>().unwrap();
        let s2 = (state2 as &dyn Any).downcast_ref::<T::StateType>().unwrap();
        self.distance(s1, s2)
    }

    fn interpolate_dyn(
        &self,
        from: &dyn AnyState,
        to: &dyn AnyState,
        t: f64,
        state: &mut dyn AnyState,
    ) {
        let from_s = (from as &dyn Any).downcast_ref::<T::StateType>().unwrap();
        let to_s = (to as &dyn Any).downcast_ref::<T::StateType>().unwrap();
        let state_s = (state as &mut dyn Any)
            .downcast_mut::<T::StateType>()
            .unwrap();
        self.interpolate(from_s, to_s, t, state_s);
    }

    fn sample_uniform_dyn(
        &self,
        rng: &mut dyn RngCore,
    ) -> Result<Box<dyn AnyState>, StateSamplingError> {
        // Adaptor because of Sized trait issue causing problems in downcasting. It's a mess!
        // TODO: Get better in Rust and find a different way to go about it.
        struct RngWrapper<'a>(&'a mut dyn RngCore);
        impl<'a> RngCore for RngWrapper<'a> {
            fn next_u32(&mut self) -> u32 {
                self.0.next_u32()
            }
            fn next_u64(&mut self) -> u64 {
                self.0.next_u64()
            }
            fn fill_bytes(&mut self, dest: &mut [u8]) {
                self.0.fill_bytes(dest)
            }
        }

        let mut wrapper = RngWrapper(rng);
        let concrete_state = self.sample_uniform(&mut wrapper)?;
        Ok(Box::new(concrete_state))
    }

    fn enforce_bounds_dyn(&self, state: &mut dyn AnyState) {
        let state_s = (state as &mut dyn Any)
            .downcast_mut::<T::StateType>()
            .unwrap();
        self.enforce_bounds(state_s);
    }

    fn satisfies_bounds_dyn(&self, state: &dyn AnyState) -> bool {
        let state_s = (state as &dyn Any).downcast_ref::<T::StateType>().unwrap();
        self.satisfies_bounds(state_s)
    }

    fn get_longest_valid_segment_length_dyn(&self) -> f64 {
        self.get_longest_valid_segment_length()
    }
}
