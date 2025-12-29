# oxmpl-js

JavaScript/WASM bindings for the OxMPL Rust library.

## Installation

> **Note:** This package is not yet published to npm. To use it, you'll need to include it as a Git submodule or clone the repository directly.

## Building

### Prerequisites

- Node.js (v22 or later recommended)
- Rust toolchain with `wasm32-unknown-unknown` target
- wasm-pack (installed automatically via npm)

### Commands

```bash
# Install dependencies and build WASM module
npm install

# Build only
npm run build

# Run tests
npm test

# Create a packaged version
npm run pack
```

The build process uses `wasm-pack` to:

- compile the Rust code to WebAssembly
- generate TypeScript definitions and JS glue
- output a fully-formed JS package in `pkg-bundler/`

## Usage

This example assumes that you've installed the package as a dependency called `oxmpl-js`.

```javascript
import oxmpl from 'oxmpl-js';

// Create a 2D state space
const space = new oxmpl.base.RealVectorStateSpace(2, [0.0, 10.0, 0.0, 10.0]);

// Define start and goal
const start = new oxmpl.base.RealVectorState([1.0, 1.0]);
const goal = new oxmpl.base.Goal({
  isSatisfied: (state) => { /* goal satisfaction check */ },
  distanceGoal: (state) => { /* distance to goal */ },
  sampleGoal: () => { /* sample goal state */ return new oxmpl.base.RealVectorState([5.0, 5.0]); }
});

// Create problem definition
const problem = oxmpl.base.ProblemDefinition.fromRealVectorState(space, start, goal);

// Define validity checker
const validityChecker = new oxmpl.base.StateValidityChecker(
  (state) => /* return true if state is collision-free */
);

// Create and run planner
const plannerConfig = new oxmpl.base.PlannerConfig(0);
const planner = new oxmpl.geometric.RRT(0.5, 0.05, problem, plannerConfig);
planner.setup(validityChecker);
const path = planner.solve(5.0);
```

## License

BSD-3-Clause License. See the main project [LICENSE](../LICENSE) file for details.
