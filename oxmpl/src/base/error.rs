// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

use std::{error, fmt};

#[derive(Debug, PartialEq)]
pub enum StateSpaceError {
    /// The length of the provided bounds does not match the dimension.
    DimensionMismatch { expected: usize, found: usize },
    /// A lower bound is greater than or equal to its corresponding upper bound.
    InvalidBound { lower: f64, upper: f64 },
    /// A 0-dimensional space was requested without explicit (empty) bounds.
    ZeroDimensionUnbounded,
}
impl fmt::Display for StateSpaceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DimensionMismatch { expected, found } => write!(
                f,
                "provided bounds length ({found}) does not match specified dimension ({expected})."
            ),
            Self::InvalidBound { lower, upper } => {
                write!(
                    f,
                    "Lower bound {lower} is greater than upper bound {upper}."
                )
            }
            Self::ZeroDimensionUnbounded => {
                write!(f, "Cannot create 0-dimensional unbounded space.")
            }
        }
    }
}
impl error::Error for StateSpaceError {}

#[derive(Debug, PartialEq)]
pub enum StateSamplingError {
    /// When trying to sample a space that isn't bounded.
    UnboundedDimension { dimension_index: usize },
    /// The space or region to be sampled has no volume (e.g., all dimensions have min >= max).
    ZeroVolume,
    /// It was not possible to generate a sample that satisfies the goal conditions.
    GoalRegionUnsatisfiable,
    /// An iterative sampling attempt failed to find a sample within a set number of attempts.
    GoalSamplingTimeout { attempts: u32 },
}
impl fmt::Display for StateSamplingError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnboundedDimension { dimension_index } => {
                write!(
                    f,
                    "Cannot sample uniformly because dimension {dimension_index} is unbounded."
                )
            }
            Self::ZeroVolume => {
                write!(f, "Cannot sample from a region with zero volume.")
            }
            Self::GoalRegionUnsatisfiable => {
                write!(f, "Could not generate a sample from the goal region because its constraints may be unsatisfiable.")
            }
            Self::GoalSamplingTimeout { attempts } => {
                write!(
                    f,
                    "Failed to generate a goal sample within {attempts} attempts."
                )
            }
        }
    }
}
impl error::Error for StateSamplingError {}

#[derive(Debug, PartialEq)]
pub enum PlanningError {
    /// Solution search timed-out.
    Timeout,
    /// No solution found.
    NoSolutionFound,
    // Planner was not setup first.
    PlannerUninitialised,
    // Start state is not valid.
    InvalidStartState,
    // State space hasn't been sampled.
    UnsampledStateSpace,
}
impl fmt::Display for PlanningError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Timeout => {
                write!(f, "No solution found within timeout.")
            }
            Self::NoSolutionFound => {
                write!(f, "No solution found.")
            }
            Self::PlannerUninitialised => {
                write!(
                    f,
                    "<Planner>.setup() was not called, thus Planner is uninitialised."
                )
            }
            Self::InvalidStartState => {
                write!(f, "Start state is not valid in the current StateSpace.")
            }
            Self::UnsampledStateSpace => {
                write!(
                    f,
                    "StateSpace is not sampled. Either Tree or Roadmap is empty."
                )
            }
        }
    }
}
impl error::Error for PlanningError {}
