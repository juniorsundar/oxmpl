// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use crate::base::state::State;

#[derive(Clone)]
pub struct CompoundState {
    pub components: Vec<Box<dyn State>>,
}

impl State for CompoundState {}
