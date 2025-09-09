// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use std::any::Any;

pub use crate::base::states::real_vector_state::RealVectorState;
pub use crate::base::states::so2_state::SO2State;
pub use crate::base::states::so3_state::SO3State;

pub trait DynClone {
    fn clone_box(&self) -> Box<dyn State>;
}

impl<T> DynClone for T
where
    T: State + Clone + 'static,
{
    fn clone_box(&self) -> Box<dyn State> {
        Box::new(self.clone())
    }
}

/// A marker trait for all state types in the planning library.
///
/// A `State` represents a single point, configuration, or snapshot of the system
/// being planned for.
///
/// Supertrait bounds:
/// - `DynClone`: States must be copyable as Dyn for runtime polymorphism.
///
/// > [!NOTE] (for self)
/// > A trait is not dyn-compatible if any of its methods return Self — unless it has a `where Self: Sized` bound.
pub trait State: DynClone + Any + 'static {}

impl Clone for Box<dyn State> {
    fn clone(&self) -> Self {
        self.clone_box()
    }
}

#[derive(Clone)]
pub struct CompoundState {
    pub components: Vec<Box<dyn State>>,
}

impl State for CompoundState {}
