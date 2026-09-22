# OxMPL

OxMPL is a sampling-based motion planning library: it finds valid paths from a start state to a goal state through a state space, while respecting validity constraints (e.g. collision).

## Language

### Path post-processing

**Path simplification**:
The umbrella for post-processing a solved path to make it shorter and/or smoother. Mirrors OMPL's `PathSimplifier`; consists of shortening and smoothing applied in that order.
_Avoid_: "optimization" (that belongs to optimal planners like RRTStar, not post-processing)

**Shortening**:
Removing states from a path by replacing sub-segments with valid direct motions. Reduces path length while keeping the path valid — but does not necessarily make it smoother.
_Avoid_: Collapsing this into "smoothing" — shortening and smoothing are different operations

**Smoothing**:
Reshaping a path (e.g. B-spline steps) to reduce jaggedness while maintaining its validity. May *increase* the number of states; assumes a metric space (triangle inequality).

**Nearest-neighbour search (NN)**:
Finding the stored state closest to a query state under the state space's metric. The search is the concept; the data structure that accelerates it (linear scan, GNAT, Kd-tree) is an implementation choice made per metric.
_Avoid_: "KdTree" or "GNAT" as the name of the feature — those are implementations, not the concept
