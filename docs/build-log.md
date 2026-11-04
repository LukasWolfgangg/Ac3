# Build log

History of changes, newest last. Specs state current values only (see `specs/`). Entries before 4 Nov 2026 were kept outside this repo (Drive) and are not reproduced here.

## 4 Nov 2026 - drag breakdown table in specs/11-performance.md regenerated

Re-ran r250c.py (chosen case, 2.80 kg, 0.44 m2, 135 mm wetted arm, 136 mm wetted rudder blade, CG 481.8 mm) and compared it with the specs; cg_layout.py and top_reach.py also reproduced their documented figures. Every performance figure (upwind, reach, downwind, takeoff, capped/uncapped, sensitivities, rudder load) matched. The drag breakdown table did not: its rudder-blade column was a constant factor 0.875 (= 119/136) below the script, i.e. it still used the old inferred 119 mm blade wetted length and was not regenerated on 30 Oct 2026 when the 136 mm length went in. Replaced the table with the r250c.py output.

| Speed | Blade drag (was -> now) | Total (was -> now) | Other column changes |
|---|---|---|---|
| 6.5 kt | 0.44 -> 0.50 N | 2.50 -> 2.54 N | foil 1.43 -> 1.41 |
| 10 kt | 0.81 -> 0.93 N | 3.86 -> 3.97 N | foil 1.77 -> 1.76, T-foil 0.50 -> 0.49 |
| 12 kt | 1.07 -> 1.22 N | 4.81 -> 4.95 N | none |
| 14.5 kt | 1.44 -> 1.65 N | 6.20 -> 6.40 N | none |

No performance figure, structural basis or decision changes: the VPP already used the 136 mm blade, only the table was stale. Not re-checked in this pass: sail_rescan.py, docnums.py, joint_straight.py.

## 4 Nov 2026 - speed cost of the straight-steel joint in specs/08 pointed to specs/11

Re-ran joint_straight.py: every joint margin matched specs/08 (glue 3.6x, steel 7.1x, 4.2x at the screw hole, carbon 14.7x above the steel's end and 5.9x or more along the bend, woven sleeve 1.3x, arm drag +4.9% at 10 kt). The speed-cost sentence in specs/08 did not: it still had the 29 Oct figures (0.10 kt at 10 and 11 kt of wind, nothing at 12 kt, top reach 22.05 against 22.15 kt). The script now gives 0.10 kt at 10 kt of wind, nothing at 11 kt, 0.2 kt at 12 kt and 0.1 kt on the top reach (21.76 against 21.86 kt), which is what specs/11 already says. Replaced the figures in specs/08 with a pointer to specs/11 so the numbers live in one place. No performance figure changed.

Not fixed, noticed: specs/08 line 60 still says the lead change leaves the all-up CG "then x 491 mm"; the Weight Budget CG is now x 481.8 mm (30 Oct 2026).
