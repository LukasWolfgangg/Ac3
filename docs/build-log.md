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

Also removed from specs/08 line 60: the parenthetical saying the lead change left the all-up CG "where it was, then x 491 mm" (stale; the Weight Budget CG is x 481.8 mm since 30 Oct 2026). The all-up CG is stated in one place, specs/08's cg_layout section.

## 4 Nov 2026 - rudder depth x taper sweep (rudder_sweep.py, analysis only)

New decision-record script rudder_sweep.py for the question of a longer rudder blade with a stronger taper. No spec, opt.py or documented number was changed; every figure below is PROPOSED and is for Lukas to confirm.

Baseline reproduced: 220 mm, 34 mm, 17%, 4 plies gives a 1.21 deg limit at 22.15 kt (rudder_limit.py: 1.2), 6.29 Nm and a 5.8 mm stock at the hull bottom (6.3 Nm, 5.8 mm), blade drag 0.92 N at 10 kt (0.93) and rake moment 1.53 Nm (1.55).

Findings (details in the script output; strip drag is a delta against the baseline because the absolute strut drag is calibrated):
- The model has no term that rewards depth, so depth was treated as a constraint. At 220 mm the T-foil is level with the main foil at the design ride (zero margin). With heel to leeward the centreline rudder rises 364.6 mm x sin(heel) relative to the foil: 36 mm at 5.7 deg (the main foil's own ventilation limit, specs/03). Staying level with the foil needs 257 mm at 5.7 deg and 0 deg pitch, 265 mm with 1 deg of bow-down pitch, 302 mm at 10 deg / 2 deg. The envelope is an assumption.
- Staying 3 T-foil chords (90 mm) under water instead needs 219 mm at 5.7 deg / 1 deg, 228 mm at 5.7 / 2 deg and 256 mm at 10 / 2 deg: the current 220 mm already meets the foil's own 5.7 deg envelope.
- Taper helps, more than expected: 220 mm with 34 -> 20 mm raises the limit to 1.60 deg, cuts blade drag 0.11 N at 10 kt and 3 g of mass, and moves upwind +0.2 to +0.3 kt in the VPP. Taper does not reduce the force at the limit (about 31-34 N), so steering authority is kept.
- Depth costs speed: about 0.5 kt upwind per +40 mm with the same taper. 240 mm with 34 -> 20 mm is level with the baseline blade on speed (0.00 kt) and on strength (1.22 deg, 4 plies, 11.8 g); 260 mm with 40 -> 22 mm costs 0.3 kt upwind, 1.27 deg with 4 plies, 15.5 g, and a 6.4 mm stock.
- Not modelled or unverified: blade ventilation, the T-foil junction on a 20-24 mm tip (the T-foil chord is 30 mm), NeuralFoil accuracy below 100k Reynolds number, load distribution along the blade (uniform per chord, conservative). Research on the real AC40 / AC75 rudder depth failed: the session's network policy blocked the pages, so no full-scale figure is used.
