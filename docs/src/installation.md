# Installation
You can use `oxmpl` in Rust, Python, or JavaScript projects.

## Supported versions

| Toolchain | Minimum |
|---|---|
| Rust | 1.85 (edition 2024) |
| Python | 3.11 |
| Node.js | 22.12 (building from source only) |

Upgrading from an earlier release? See [Migration Guides](migration/index.md).

## Rust
The core library is available on crates.io and can be added to your project's `Cargo.toml`:

```toml
[dependencies]
oxmpl = "0.8" # Replace with the latest version
```

## Python
The library is available on PyPI and can be installed with `pip` (Python 3.11 or later):

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

### Prerequisites
- Rust 1.85+ (edition 2024)

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
- Python 3.11+
- `maturin` 1.15+ (install via `pip install "maturin>=1.15"`)

### Commands
```bash
# Build and install in the current environment
maturin develop -m oxmpl-py/Cargo.toml
```

## JavaScript / WASM
If you need to build the JavaScript bindings from source:

### Prerequisites
- Node.js 22.12+
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
