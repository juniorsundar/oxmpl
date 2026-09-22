# Core — OxMPL source map and invariants

Sampling-based motion planning library in Rust, inspired by OMPL. Workspace members: `oxmpl` (core Rust crate), `oxmpl-py` (pyo3 bindings), `oxmpl-js` (wasm-bindgen bindings), `docs` (mdBook + custom `oxmpl-docs` preprocessor binary).

## Source map
- `oxmpl/src/base/` — shared foundations: `states/` (RealVector, SO2, SO3, SE2, SE3, Compound, Any), `spaces/` (matching state spaces), `goal.rs`, `validity.rs`, `planner.rs` (core `Planner` trait + `Path` newtype), `problem_definition.rs`, `error.rs`.
- `oxmpl/src/geometric/planners/` — RRT, RRTConnect, RRTStar, PRM.
- Bindings mirror core layout: `oxmpl-py/src/{base,geometric}/`, `oxmpl-js/src/{base,geometric}/` — keep parity.
- `docs/src/` — mdBook sources (`SUMMARY.md`), including contributor guides for implementing planners/state spaces.
- **Planning lives OUTSIDE the repo** (decided 2026-09-22): the repo must stay free of management content. Sprint/epic/story tracking = TaskNotes MCP (system of record, vault at ~/Dropbox/vault); narrative planning files (README, roadmap, sprint pages, backlog parking lot, issues queue) at `~/Dropbox/vault/pages/oxmpl/planning/`. `docs/BACKLOG.md` was deleted from the repo (successor: the vault backlog.md).
- **TaskNotes conventions for oxmpl**: everything OxMPL-related (tasks and vault notes) carries the `oxmpl` tag. Hierarchy is encoded via `projects` (user-set 2026-09-22): epics are TASKS (tags `epic`, `epic/E<N>`) whose `projects` = superproject `[[`oxmpl` - A Rust-based Motion Planning Library]]`; stories are tasks whose `projects` = their epic's note link (e.g. `[[E1 Release 0.7.0 — Stability & Robustness]]`), never the superproject directly. Stories carry `epic/E<N>`, `size-S/M/L`, `sprint-NNN` tags.
- **PM persona has a session-bootstrap ritual**: at the start of every PM session, reconstruct the board from `~/Dropbox/vault/pages/oxmpl/planning/sprints/` (current sprint = dates contain today, else lowest NNN without a retro) + TaskNotes queries; report the board before asking anything. Procedure lives in the persona prompt, state is always derived — never hardcoded. TaskNotes is the status of record; sprint pages are narrative.
- **TaskNotes query quirk**: tag filters must use the `contains` operator — `is` silently returns nothing for tags. Status filters work with `is`. Sprint 001 (2026-09-22 → 09-28) committed all of E1 (stories S1.1–S1.5, goal: release 0.7.0); sprint pages live in planning/sprints/ named `` `oxmpl` - sprint-NNN.md ``.
- **Vault page naming convention** (user-set): oxmpl planning pages are named `` `oxmpl` - <name>.md `` — e.g. `` `oxmpl` - planning.md `` (conventions), `` `oxmpl` - roadmap.md ``, `` `oxmpl` - backlog.md ``, `` sprints/`oxmpl` - sprints.md `` — and future sprint pages will be `` `oxmpl` - sprint-NNN.md ``. Pages carry frontmatter (tags, created, dg-publish).
- `docs/planning/adr/` — stays IN the repo (user decision): ADRs + root `CONTEXT.md` are technical docs. See Planning workflow below.
- `docs/planning/adr/` — ADRs (user moved them here from the usual `docs/adr/`); consumer rules in `docs/agents/domain.md`. `CONTEXT.md` (root, if created) + these ADRs form the domain docs.
- `AGENTS.md` (repo root) — `## Agent skills` block pointing at `docs/agents/*.md`.

## Invariants
- Every core feature must land in: Rust core + BOTH bindings + integration tests + docs. Scope estimates ignoring this multiplier are wrong.
- Core tests are integration tests in `oxmpl/tests/`, one file per planner×state-space combo (`<planner>_<space>_tests.rs`) — a new state space or planner must extend the whole matrix.
- Releases are semver via release-please conventional commits (single `simple` release for the whole workspace; version pinned in root `Cargo.toml`).
- `Path<S>` newtype wraps `Vec<S>`; `Planner` trait is generic over `S: State, SP: StateSpace<StateType = S>, G: Goal<S>` with `configure()` required before `solve()`.

## Project workflow (personas)
- `.pi/personas/` contains `senior-engineer` (release/scope planning) and `project-manager` (epics/stories/sprints in `docs/planning/`). Switch via `/persona <name>`.
- Read `mem:conventions` for code style; `mem:suggested_commands` / `mem:task_completion` for build/test commands.
