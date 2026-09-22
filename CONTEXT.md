# OxMPL

OxMPL is a sampling-based motion planning library: it finds valid paths from a start state to a goal state through a state space, while respecting validity constraints (e.g. collision).

## Language

### Validity

**State validity**:
Whether a single state is acceptable (e.g. collision-free, within constraints). Judged by a state validity checker.

**Motion**:
The continuous transition from one state to another, as traced by the state space's interpolation. A motion is a curve in the space, not necessarily a straight line.
_Avoid_: "edge" (a roadmap/graph term), "segment" (reserved for the resolution subdivision below)

**Motion validation**:
Deciding whether every state along a motion is valid, assuming the motion's start state is already valid. Distinct from state validity: a motion between two valid states can still be invalid.
_Avoid_: "collision checking" (collision is one validity criterion, not the concept)

**Resolution**:
The space's own statement of how finely motions must be inspected: the longest valid segment, a length below which a motion is assumed valid if its endpoints are. Owned by the state space, not by any validator.

**Space information**:
The state space together with the means of judging validity in it: the state validity checker and the motion validator. A planning problem is posed over a space information.
_Avoid_: "environment", "world" (the obstacles a checker consults are the user's concern, not this concept)

**Checking strategy**:
How a particular motion validator decides a motion is valid: discrete inspection at the space's resolution, an analytic/continuous check, a batched check, etc. Owned by the validator; a validator may ignore the space's resolution entirely.

### Roadmaps

**Milestone**:
A valid state that has been added to a roadmap (PRM). Samples rejected as invalid are not milestones.
_Avoid_: "node" or "vertex" when the roadmap meaning is intended

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
