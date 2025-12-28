// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

import * as oxmpl from './pkg-bundler/oxmpl_js.js';

export const base = {
  Goal: oxmpl.Goal,
  Path: oxmpl.Path,
  PlannerConfig: oxmpl.PlannerConfig,
  ProblemDefinition: oxmpl.ProblemDefinition,
  RealVectorState: oxmpl.RealVectorState,
  RealVectorStateSpace: oxmpl.RealVectorStateSpace,
  SO2State: oxmpl.SO2State,
  SO2StateSpace: oxmpl.SO2StateSpace,
  SO3State: oxmpl.SO3State,
  SO3StateSpace: oxmpl.SO3StateSpace,
  SE2State: oxmpl.SE2State,
  SE2StateSpace: oxmpl.SE2StateSpace,
  SE3State: oxmpl.SE3State,
  SE3StateSpace: oxmpl.SE3StateSpace,
  StateValidityChecker: oxmpl.StateValidityChecker,
};

export const geometric = {
  PRM: oxmpl.PRM,
  RRT: oxmpl.RRT,
  RRTConnect: oxmpl.RRTConnect,
  RRTStar: oxmpl.RRTStar,
};

export default {
  base,
  geometric,
};
