# Introduction
`oxmpl` is a sampling-based motion planning library written in Rust, inspired by the structure and concepts of the [Open Motion Planning Library (OMPL)](http://ompl.kavrakilab.org/).

It is **NOT** OMPL with Rust bindings.

It provides a flexible and type-safe Rust API for core planning algorithms and offers high-level Python and JavaScript/WASM bindings for rapid prototyping and integration into diverse robotics projects.

## Key Features
* **Safe & Fast**: Built in Rust for memory safety and performance.
* **Extensible Architecture**: Core concepts like `StateSpace`, `StateValidityChecker`, and `Planner` are defined as traits, making the library highly extensible.
* **Pythonic Bindings**: A simple, easy-to-use Python API powered by PyO3 that allows users to define problem-specific logic (like collision checkers) in pure Python.
* **JavaScript/WASM Bindings**: Run your motion planning logic in the browser or Node.js with high performance.
* **Inspired by OMPL**: Follows the modular design of OMPL, making it familiar to those in the robotics community.
