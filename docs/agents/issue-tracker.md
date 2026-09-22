# Issue tracker: Local Markdown (personal vault)

Issues and specs for this repo live as markdown files in the developer's personal vault, **outside this repository**: `~/Dropbox/vault/pages/oxmpl/planning/issues/`.

> This location is machine-local: these files are not part of the repo clone and are not visible to external contributors. Planning artifacts (roadmap, sprints, backlog) and ticket tracking (TaskNotes) live in the same vault directory — see `` ~/Dropbox/vault/pages/oxmpl/planning/`oxmpl` - planning.md ``.

## Conventions

- One feature per directory: `~/Dropbox/vault/pages/oxmpl/planning/issues/<feature-slug>/`
- The spec is `~/Dropbox/vault/pages/oxmpl/planning/issues/<feature-slug>/spec.md`
- Implementation issues are one file per ticket at `~/Dropbox/vault/pages/oxmpl/planning/issues/<feature-slug>/issues/<NN>-<slug>.md`, numbered from `01`, never a single combined tickets file
- Triage state is recorded as a `Status:` line near the top of each issue file (see `triage-labels.md` for the role strings)
- Comments and conversation history append to the bottom of the file under a `## Comments` heading

## Relationship to sprint planning

Sprint planning (epics, stories, sprints) is maintained by the `project-manager` persona in the same vault directory: roadmap and sprint files in `~/Dropbox/vault/pages/oxmpl/planning/`, ticket statuses in TaskNotes. Issues here are the inbound queue: when a story enters a sprint, its source issue file should be linked from the sprint page.

## External issues

This repo is public on GitHub (github.com/juniorsundar/oxmpl). External issues filed on GitHub are **not** tracked automatically: mirror any that matter into the vault issues directory by hand, noting the upstream `#issue-number` in the file.

## When a skill says "publish to the issue tracker"

Create a new file under `~/Dropbox/vault/pages/oxmpl/planning/issues/<feature-slug>/` (creating the directory if needed).

## When a skill says "fetch the relevant ticket"

Read the file at the referenced path. The user will normally pass the path or the issue number directly.

## Wayfinding operations

Used by `/wayfinder`. The **map** is a file with one **child** file per ticket.

- **Map**: `~/Dropbox/vault/pages/oxmpl/planning/issues/<effort>/map.md` (the Notes / Decisions-so-far / Fog body).
- **Child ticket**: `~/Dropbox/vault/pages/oxmpl/planning/issues/<effort>/issues/NN-<slug>.md`, numbered from `01`, with the question in the body. A `Type:` line records the ticket type (`research`/`prototype`/`grilling`/`task`); a `Status:` line records `claimed`/`resolved`.
- **Blocking**: a `Blocked by: NN, NN` line near the top. A ticket is unblocked when every file it lists is `resolved`.
- **Frontier**: scan the effort's `issues/` directory for files that are open, unblocked, and unclaimed; first by number wins.
- **Claim**: set `Status: claimed` and save before any work.
- **Resolve**: append the answer under an `## Answer` heading, set `Status: resolved`, then append a context pointer (gist + link) to the map's Decisions-so-far in `map.md`.
