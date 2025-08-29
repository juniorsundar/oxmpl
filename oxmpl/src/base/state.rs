// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

pub use crate::base::states::real_vector_state::RealVectorState;
pub use crate::base::states::so2_state::SO2State;
pub use crate::base::states::so3_state::SO3State;

/// A marker trait for all state types in the planning library.
///
/// A `State` represents a single point, configuration, or snapshot of the system
/// being planned for.
///
/// Supertrait bounds:
/// - `Clone`: States must be copyable.
pub trait State: Clone {}
