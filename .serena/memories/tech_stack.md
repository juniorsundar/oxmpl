# Tech stack

- **Core**: Rust, edition 2021, workspace `resolver = 2`, version pinned in root `Cargo.toml` (`x-release-please-version` marker). Release profile: `opt-level = "s"`, LTO on.
- **Python bindings**: pyo3 + maturin (`oxmpl-py/pyproject.toml`, feature `pyo3/extension-module`), tests via pytest in `oxmpl-py/tests/`, venv at `.venv/` (activated via direnv `.envrc`).
- **JS bindings**: wasm-bindgen/wasm-pack style npm package in `oxmpl-js/` (index.js/index.d.ts, pkg-bundler), tests via vitest, lint via eslint, format via prettier.
- **Docs**: mdBook; `docs` is a workspace crate providing the `oxmpl-docs` binary (custom preprocessor). Built book is `docs/book` (gitignored). Hosted via GitHub Pages.
- **Tooling**: `just` task runner (see `mem:suggested_commands`), Nix flake + direnv, pre-commit hooks (rustfmt, clippy `-D warnings`, cargo check, eslint, prettier).
- **Releases/CI**: release-please with conventional commits; workflows: rust.yml, pypi.yml, js.yml, docs.yml, release-please.yml.
- **Versioning pins that matter**: `requires-python = ">=3.8"`, maturin `>=1.8,<2.0`.
