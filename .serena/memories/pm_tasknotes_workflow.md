# PM workflow: TaskNotes task gates & MCP quirks

**Always implement task gates via TaskNotes' `blockedBy` dependency field — never leave "blocked by X" as prose only.** Gates are machine-readable state: the plugin computes `isBlocked`/`isBlocking`/`blocking` and surfaces them in views (kanban, agenda, relationships).

## Vault facts
- Task files: `~/Dropbox/vault/task_notes/tasks/` (NOT in the repo — planning/management content never enters the repo, see `mem:conventions`).
- Filenames = task titles with `:` stripped (e.g. title "S1.5 — Release chores: publish 0.7.0" → file `S1.5 — Release chores publish 0.7.0.md`).

## blockedBy format (frontmatter YAML)
```yaml
blockedBy:
  - uid: "[[Exact task filename without .md]]"
    reltype: FINISHTOSTART
```
Insert inside the `---` frontmatter of the blocked task. Wikilink uid must match the filename exactly.

## Gate policy (established 2026-09-22, oxmpl E1)
- **Story gates**: a story that cannot start until others finish lists ALL its upstream dependencies, including already-done ones (done deps don't block; keeps the gate definition complete and auditable). Example: S1.5 blockedBy S1.1–S1.4.
- **Epic gates**: epics list ALL their stories as blockedBy (E1 ← S1.1–S1.5, E2 ← S2.1–S2.2). Epic closes when its stories close.
- New stories that gate on others must get `blockedBy` wired at creation time.
- After editing `blockedBy`, verify via `tasknotes_get_task`: `isBlocked: true` + `blocking` on the upstream task confirms the graph is recognized (bidirectional).

## TaskNotes MCP tool quirks (important when scripting tasks)
- **Task id format** (cost hours 2026-09-22): `get_task`/`update_task`/time-tracking ids are **vault-relative paths** like `task_notes/tasks/<filename>.md` — the `task_notes/` prefix is REQUIRED. `tasks/<file>.md` and bare filenames return "Task not found". Not a stale index; a wrong-path error.
- `tasknotes_create_task` **drops the `description` param** (returns empty details) and **defaults `scheduled` to today** when null. Always follow a create with `tasknotes_update_task` (correct id format!) to set details + `scheduled: null` if unscheduled.
- `blockedBy` is **not exposed** by any tasknotes MCP tool — edit the task file's frontmatter directly, then verify with `get_task`.
- After `tasknotes_update_task` on a gated task, re-check that `blockedBy` survived (unverified whether updates preserve custom frontmatter fields — confirm if it ever disappears).
- `tasknotes_update_task` accepts full `details` markdown; keep the story structure: Intent / Acceptance criteria / Tasks (+ Completion record when closing, with evidence).
- Story tag conventions: `task, oxmpl, epic/E<N>, size-{S,M,L}, sprint-00N`; epics: `task, oxmpl, epic, epic/E<N>`. Sprint tags mark *intent* — dates get set only at sprint planning.

## Sprint rituals checklist
- Close-out: tick all checklists, record grep/test evidence as completion record, set status done.
- File follow-ups as stories under the right epic (don't leave work under the super project `[[`oxmpl` - A Rust-based Motion Planning Library]]` — that's a parking lot, not a home).
- Sprint planning: assign `sprint-00N` tags + scheduled/due dates; roadmap/sprint narrative notes in vault pages get updated then.
