# Task completion checklist

A coding task is done when ALL pass:

1. `cargo fmt --all` — formatting
2. `cargo clippy --workspace -- -D warnings` — zero lints
3. `cargo check --workspace` — compiles (workspace = core + both binding crates + docs crate)
4. `just test-all` — Rust integration tests + pytest + vitest
5. If public API changed: `just test-examples` (examples in all three languages must still run) AND check whether docs (`docs/src/*.md`, contributor guides) need updates
6. If new planner/state-space combo: added integration test files to the full matrix, in all three languages
7. `pre-commit run --all-files` — clean (covers 1–3 + eslint/prettier)

Commit messages: conventional commits (feat/fix/…) for release-please.
Sprint work: update the current sprint file in `docs/planning/sprints/` and story status in `docs/planning/roadmap.md`.
