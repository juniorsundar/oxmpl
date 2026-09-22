# Two-tier state traits: lightweight `State` marker + opt-in `AnyState` for dynamic dispatch

Earlier, `State` required `DynClone + Any` so that `CompoundState` could hold heterogeneous substates behind trait objects, and SE2/SE3 were implemented as `CompoundState` wrappers. This forced every typed state to carry dynamic-dispatch machinery (vtable cloning, RTTI) even for the common fully-typed planning path, and made the rigid-body spaces round-trip through compound composition.

We decided to split the trait in two: `State` is a lightweight static marker (`Debug + Send + Sync + 'static`) for typed planning through generic state spaces, while `AnyState` carries the dynamic capabilities (`clone_box`, `as_any` downcasting) and is implemented only by types that participate in compound-state composition. SE2/SE3 became direct typed structs rather than compound wrappers.

Consequence: two parallel traits exist and implementers must choose; the dynamic path is opt-in rather than mandatory, and the typed path stays minimal. This split is already public API across Rust, Python, and JS bindings, so reversing it costs a breaking release.
