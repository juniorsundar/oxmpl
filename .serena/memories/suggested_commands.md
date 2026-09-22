# Suggested commands

`just` is the entry point (see `Justfile`):
- `just build-all` / `just build-rust` / `just build-py` / `just build-js`
- `just test-all` / `just test-rust` (`cargo test -p oxmpl`) / `just test-py` / `just test-js`
- `just test-examples` — runs ALL examples in all three languages; use after changing public APIs
- `just docs` — `mdbook build docs`; for live preview: `cd docs && mdbook serve --open`

Direct:
- Python venv lives at `.venv/` (direnv auto-activates); py bindings need `maturin develop -m oxmpl-py/Cargo.toml` before pytest.
- JS: `cd oxmpl-js && npm run build` (wasm-pack into `pkg-bundler`), `npm test` (vitest), `npm run lint`.
- Pre-commit: `pre-commit run --all-files`.

System is Linux; `nix`/`direnv` present via flake.
