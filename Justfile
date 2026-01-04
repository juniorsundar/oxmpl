set shell := ["bash", "-c"]

default:
    @just --list

# Build
# ==========================================

build-rust:
    cargo build -p oxmpl

build-py:
    source .venv/bin/activate && maturin develop -m oxmpl-py/Cargo.toml

build-js:
    cd oxmpl-js && npm install && npm run build

build-all: build-rust build-py build-js

# Test
# ==========================================

test-rust:
    cargo test -p oxmpl

test-py:
    source .venv/bin/activate && pytest oxmpl-py/tests

test-js:
    cd oxmpl-js && npm test

test-all: test-rust test-py test-js

# Examples
# ==========================================

example-rust:
    for file in oxmpl/examples/*.rs; do \
        name=$(basename $file .rs); \
        echo "Running Rust example: $name"; \
        cargo run --example $name -p oxmpl || exit 1; \
    done

example-py:
    source .venv/bin/activate && \
    for file in oxmpl-py/examples/*.py; do \
        echo "Running Python example: $file"; \
        python $file || exit 1; \
    done

example-js:
    cd oxmpl-js && \
    for file in examples/*.js; do \
        echo "Running JS example: $file"; \
        node --experimental-wasm-modules $file || exit 1; \
    done

test-examples: example-rust example-py example-js

# Documentation
# ==========================================

docs:
    mdbook build docs
