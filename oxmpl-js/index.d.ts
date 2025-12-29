// Copyright (c) 2025 Junior Sundar
//
// SPDX-License-Identifier: BSD-3-Clause

import * as oxmpl from './pkg-bundler/oxmpl_js';

export namespace base {
  export import CompoundState = oxmpl.CompoundState;
  export import CompoundStateBuilder = oxmpl.CompoundStateBuilder;
  export import CompoundStateSpace = oxmpl.CompoundStateSpace;
  export import CompoundStateSpaceBuilder = oxmpl.CompoundStateSpaceBuilder;
  export import Goal = oxmpl.Goal;
  export import Path = oxmpl.Path;
  export import PlannerConfig = oxmpl.PlannerConfig;
  export import ProblemDefinition = oxmpl.ProblemDefinition;
  export import RealVectorState = oxmpl.RealVectorState;
  export import RealVectorStateSpace = oxmpl.RealVectorStateSpace;
  export import SO2State = oxmpl.SO2State;
  export import SO2StateSpace = oxmpl.SO2StateSpace;
  export import SO3State = oxmpl.SO3State;
  export import SO3StateSpace = oxmpl.SO3StateSpace;
  export import SE2State = oxmpl.SE2State;
  export import SE2StateSpace = oxmpl.SE2StateSpace;
  export import SE3State = oxmpl.SE3State;
  export import SE3StateSpace = oxmpl.SE3StateSpace;
  export import StateValidityChecker = oxmpl.StateValidityChecker;
}

export namespace geometric {
  export import PRM = oxmpl.PRM;
  export import RRT = oxmpl.RRT;
  export import RRTConnect = oxmpl.RRTConnect;
  export import RRTStar = oxmpl.RRTStar;
}

declare const _default: {
  base: typeof base;
  geometric: typeof geometric;
};

export default _default;
