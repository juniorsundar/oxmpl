# Installation
You can use `oxmpl` in Rust, Python, or JavaScript projects.

## Rust
The core library is available on crates.io and can be added to your project's `Cargo.toml`:

```toml
[dependencies]
oxmpl = "0.4.0" # Replace with the latest version
```

## Python
The library is available on PyPI and can be installed with `pip`:

```bash
pip install oxmpl-py
```

## JavaScript / WASM
JavaScript/WASM bindings are available:

```bash
npm install oxmpl-js
```

# Building from Source
If you want to use the latest features or contribute to the project, you can build the libraries from source.

## Rust
To build the core Rust library:

```bash
# Clone the repository
git clone https://github.com/juniorsundar/oxmpl.git
cd oxmpl

# Build
cargo build --release
```

## Python
To build the Python bindings (`oxmpl-py`):

### Prerequisites
- Python 3.8+
- `maturin` (install via `pip install maturin`)

### Commands
```bash
# Build and install in the current environment
maturin develop -m oxmpl-py/Cargo.toml
```

## JavaScript / WASM
If you need to build the JavaScript bindings from source:

### Prerequisites
- Node.js (v22 or later recommended)
- Rust toolchain with `wasm32-unknown-unknown` target
- `wasm-pack` (installed automatically via `npm install`)

### Commands
```bash
cd oxmpl-js

# Install dependencies and build WASM module
npm install

# Build only
npm run build

# Create a packaged version
npm run pack
```
