# Conventions

- **License headers**: every Rust source file starts with `// Copyright (c) 2025 Junior Sundar` + `// SPDX-License-Identifier: BSD-3-Clause`.
- **Module layout**: `base` holds shared foundations (states, spaces, goal, validity, planner trait); algorithm-specific code goes under `geometric/` (future control-based planners would get their own sibling). Bindings mirror the core layout exactly (`base`, `geometric` submodules in both `oxmpl-py/src` and `oxmpl-js/src`).
- **Generics pattern**: planners are generic over `S: State, SP: StateSpace<StateType = S>, G: Goal<S>`; state spaces expose associated type `StateType`. New state/spaces implement the paired `State`/`StateSpace` traits and register in `states/mod.rs` + `spaces/mod.rs`.
- **Testing**: integration tests only (no unit tests inside src), one file per planner×state-space combo in `oxmpl/tests/`, named `<planner>_<space>_tests.rs` (rvss, so2ss, so3ss, se2ss, se3ss, css abbreviations). New planner or space ⇒ extend the full matrix in all three languages (Rust + pytest + vitest).
- **Planner config**: planner-specific params travel in a `PlannerConfig` struct (pattern established in JS bindings too).
- **Docs**: public traits/structs get full rustdoc (`///` summaries with usage notes). Contributor guides in `docs/src/contributing/` describe how to add planners/state spaces — update them when extending abstractions.
- **Commits**: conventional commits (feat/fix/docs/style/refactor/perf/test/build/ci) — they feed release-please and the changelog.
- **Examples**: each binding has runnable examples (`oxmpl/examples/*.rs`, `oxmpl-py/examples/*.py`, `oxmpl-js/examples/*.js`) — add one per user-facing feature.
- **ADRs & issue tracker**: ADRs live in the repo at `docs/planning/adr/` (NOT the conventional `docs/adr/`). Issues/specs live OUTSIDE the repo at `~/Dropbox/vault/pages/oxmpl/planning/issues/<slug>/` — see `docs/agents/` and `AGENTS.md`. Planning/management content never enters the repo.
- **PM workflow**: task/sprint/epic management runs via TaskNotes in the vault — ALWAYS wire task gates with `blockedBy` frontmatter (format + MCP quirks in `mem:pm_tasknotes_workflow`).
- **Code search tooling** (user instruction 2026-09-22): use the Serena MCP for codebase searches/inspection — `serena_find_symbol`, `serena_find_referencing_symbols`, `serena_search_for_pattern`, `serena_get_symbols_overview` — instead of bash grep/sed. Bash is for running builds/tests, not for reading code.
