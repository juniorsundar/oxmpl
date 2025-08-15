# Changelog

## [0.3.0](https://github.com/juniorsundar/oxmpl/compare/v0.2.0...v0.3.0) (2025-08-15)


### Features

* Implement PRM in Python ([50b3f69](https://github.com/juniorsundar/oxmpl/commit/50b3f6913e5cc2b4a290a95002fbdb1e2d4be9ce))
* Implement PRM in Rust w/ Integration tests ([1add7d4](https://github.com/juniorsundar/oxmpl/commit/1add7d41c0f0d9d4516f9f8d58dffe9b163dc7a8))
* Implement RRTConnect in Rust ([ba594b9](https://github.com/juniorsundar/oxmpl/commit/ba594b97559e253402e2f96165190845d041fa41))
* Implement RRTStar in Python ([a70ab5d](https://github.com/juniorsundar/oxmpl/commit/a70ab5d60fb1e501ef5f28a0df2371224aea295c))
* Implement RRTStar in Rust w/ integration tests ([1642704](https://github.com/juniorsundar/oxmpl/commit/1642704659d861bd2d6924b4e4970feee821bd0e))


### Bug Fixes

* PyO3 bindings for PRM ([57d9e82](https://github.com/juniorsundar/oxmpl/commit/57d9e823098f146a24bec5a554c46af82a9fc242))


### Code Refactoring

* Module structure for import similar to C++ ([ba58429](https://github.com/juniorsundar/oxmpl/commit/ba5842975e15e0ca89d000c10c4a8ff5a2621423))
* Move RRTConnect helper functions as associated functions ([736cc1d](https://github.com/juniorsundar/oxmpl/commit/736cc1d749449232debce32763f3fc320ed62d76))


### Tests

* Implement RRTConnect integration test ([d67dbbb](https://github.com/juniorsundar/oxmpl/commit/d67dbbbc60f074ba9558074d3e0ee53406be4f1b))

## 0.2.0 (2025-06-23)


### chore

* release 0.2.0 ([7b0146d](https://github.com/juniorsundar/oxmpl/commit/7b0146d3066916293a6e2627ee7bb83fe773b98e))


### Features

* Bound checking in RealVectorStateSpace ([7a78c0a](https://github.com/juniorsundar/oxmpl/commit/7a78c0a595d542717f20fc47b92cccf47446596f))
* Implement RealVectorStateSpace and basic structure ([b42d9e8](https://github.com/juniorsundar/oxmpl/commit/b42d9e8ccba67afaf6ff1576a0c42f071f820600))
* Implementing SO2StateSpace ([a0ea17c](https://github.com/juniorsundar/oxmpl/commit/a0ea17cf48e6f70996ef01f53de0e121207ad9f3))
* **planners:** Implement basic RRT ([f8828e9](https://github.com/juniorsundar/oxmpl/commit/f8828e9f6f87603e1ddf82ba12d227829ad06728))
* **python:** Implemented initial bindings for RRT ([c563846](https://github.com/juniorsundar/oxmpl/commit/c563846420ae630798adf956e2d9605f05cf5155))


### Bug Fixes

* Better error handling in RealVectorStateSpace::new ([ec37a78](https://github.com/juniorsundar/oxmpl/commit/ec37a78a1c550494c5bc4cae529c55e5503186a0))


### Documentation

* Added Documentation and DocTests + License header ([#1](https://github.com/juniorsundar/oxmpl/issues/1)) ([f43319f](https://github.com/juniorsundar/oxmpl/commit/f43319fe6f437d1388d465e3c744b41bddb9f3e0))
* **oxmpl-py:** Added documentation for Python Bindings ([#6](https://github.com/juniorsundar/oxmpl/issues/6)) ([7621a71](https://github.com/juniorsundar/oxmpl/commit/7621a71d4cfd538a4a21adb9f53abfdec742aa2d))
* **README:** Updated initial README.md ([#7](https://github.com/juniorsundar/oxmpl/issues/7)) ([0237de0](https://github.com/juniorsundar/oxmpl/commit/0237de0b83142159328f8af8acd0746d31951b62))


### Code Refactoring

* Split python project out and state spaces into separate module ([5387441](https://github.com/juniorsundar/oxmpl/commit/53874418fc35d7279c8ec2f262c5d1e1257e1cf8))


### Tests

* RRT basic integration test ([e081aef](https://github.com/juniorsundar/oxmpl/commit/e081aef671b8686146e8971a72cadcddaddb0555))


### Build System

* Added required fields to Cargo ([a6a239d](https://github.com/juniorsundar/oxmpl/commit/a6a239d198a0aa2bc45b08e14b3639dcdb66d715))


### Continuous Integration

* Added root level Cargo.toml ([deca994](https://github.com/juniorsundar/oxmpl/commit/deca994362132e1333a1f8ed84e7fd92d2944aba))
* Create release-please-ci ([#2](https://github.com/juniorsundar/oxmpl/issues/2)) ([abcaed8](https://github.com/juniorsundar/oxmpl/commit/abcaed827a2b8997c8479a6cfab22d06018e33f1))
* create rust.yml ([f8f90ee](https://github.com/juniorsundar/oxmpl/commit/f8f90ee7147bb772ea3580430b886f65b5e582dc))
* Fix tests and build ([2943f51](https://github.com/juniorsundar/oxmpl/commit/2943f51809cdd06906e18ba1d1a418475223de89))
* Implement pre-commit hooks ([#5](https://github.com/juniorsundar/oxmpl/issues/5)) ([6b5fa13](https://github.com/juniorsundar/oxmpl/commit/6b5fa133f679743bf8fd8ff15a82aa04614115e2))
* Updated tagging CI ([1ace08b](https://github.com/juniorsundar/oxmpl/commit/1ace08b3b03e906845b7bdb390784bdd3fe2521c))
