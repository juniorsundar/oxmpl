// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use std::fmt::Debug;

pub use crate::base::states::{
    any_state::AnyState, compound_state::CompoundState, real_vector_state::RealVectorState,
    se2_state::SE2State, se3_state::SE3State, so2_state::SO2State, so3_state::SO3State,
};

/// A marker trait for all typed state types in the planning library.
///
/// A `State` represents a single point, configuration, or snapshot of the system being planned.
/// This is a lightweight static trait — it only requires the minimum bounds needed for typed
/// planning through generic state spaces.
///
/// For runtime type inspection, downcasting, and trait-object cloning, use the [`AnyState`]
/// trait instead. Only types that participate in compound-state composition (e.g., as components
/// of a [`CompoundState`]) need to implement `AnyState`.
pub trait State: Debug + Send + Sync + 'static {}
