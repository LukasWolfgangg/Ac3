# RC sailboat build
Migrated from Google Drive ("RC Sailboat Build" folder) to a repo so scripts, specs and history live together.

## Finish the migration (5 min, by hand)
1. Drive > "Simulation & Analysis" folder > right-click > Download. Unzip the .py/.csv files into `scripts/` (arm_pts.csv into `data/`).
2. For each Google Doc: File > Download > Markdown (.md). Put them in `docs/` as named in CLAUDE.md. Design & Specs is ~40 KB: split it by section (## 0 ... ## 11) into `docs/specs/` once, so Claude Code does not load all of it each session.
3. Weight Budget, Materials & BOM, Build Task Tracker: File > Download > CSV into `data/`.
4. `cg_layout.py` reads a snapshot of the Weight Budget; point it at the CSV in `data/`.
5. Open this folder in Claude Code. Then decide whether Drive stays as a read-only mirror or is retired (do not edit both).

## Not carried over
Fusion 360 and foil.tools connectors, Drive connector, chat memory. Set those up as MCP servers in Claude Code if wanted.
