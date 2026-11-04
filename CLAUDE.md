# RC foiling sailboat (AC40-style monohull, 1000 mm LOA)

Owner: Lukas. Direct, no flattery: if a decision or number looks wrong, say so and explain why.

## Project in one paragraph
AC40-style foiling RC monohull, active flight control (PID + IMU + sonar). Target: 10-12 kt true wind, ~10 kt boat speed upwind, foil-borne from <= 8.75 kt of wind. Single leeward foil at a time, cant arms (reach cap 250 mm), steel wet foil unit + carbon upper arm. Current design state lives in the `specs/` files (single source of truth, split from the Drive doc "Design & Specs").

## Layout
- `specs/` - source of truth for every spec (00 concept + decision log, 01-02 dimensions/hull, 03 foils, 04-05 rudder/rig, 08 open confirmations, 09-10 weight + risks, 11 performance). Other docs POINT to it, never restate numbers. Section 6-7 (flight control) is in the Drive doc 'Flight Control Electronics - Build Plan'.
- `docs/build-log.md` - history of changes. Specs state current values only.
- `docs/mechanisms/` - foil arm, rudder, hydrodynamic loads, rig & sail.
- `docs/tests/` - bench-test methods.
- `scripts/` - Python analysis (opt.py is the VPP engine; r250c.py chosen case; top_reach.py structural basis; cg_layout.py; joint_straight.py). See `scripts/README.md`.
- `data/` - arm_pts.csv, weight budget / BOM exports (CSV).
- `cad/` - notes only. Fusion 360 files stay in Fusion cloud; Hull_v2 is master CAD for hull, arm, gimbal. Foilarm_v5 and Gimbal D are frozen records, do not edit.
- `firmware/` - Arduino (Nano, MPU6050, HC-SR04). Arduino IDE 2.3.10 on Linux.

## Status tags (keep them)
DECIDED = Lukas's decision. PROPOSED = derived by analysis, awaiting his confirmation. OPEN = unresolved. Never promote PROPOSED to DECIDED yourself.

## Working rules
1. All numbers must be real and checked. Estimates are starting points only; validate structurally and aerodynamically before locking in.
2. 2D foil polars are insufficient at this scale. Validate with 3D (foil.tools AeroBuildup/VLM).
3. If a prior method is found wrong, recompute ALL downstream numbers before proceeding.
4. Each fact lives in exactly one place. Change it in specs/, point elsewhere. Remove outdated docs rather than keeping them alongside.
5. Weight changes are justified by a specific construction or analysis decision.
6. Use actual airfoil coordinates for thin-shell inertia, not simplified sections.
7. Sanity-check proportions against the full-scale AC75/AC40.
8. Prefer mechanically simple, easy-to-manufacture solutions; manufacturing complexity is a reason to rework.
9. Flag known weak spots rather than hiding them: strut-drag calibration constant (CAL 0.716) is anchored to a retired 18.4 N figure; real drag is the dominant unknown (measure with on-water speed runs).
10. When a script output changes a documented number, update the doc in the same change and add a build-log entry.

## Workflow for a change
edit script/spec -> re-run chain (opt.py -> r250c.py -> top_reach.py -> cg_layout.py as relevant) -> compare to documented figures -> update specs/ -> append build-log.
