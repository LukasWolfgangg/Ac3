# Design & Specs - 11. Performance: thrust-versus-drag balance

Source: Google Doc "Design & Specs" (30 Oct 2026). Status tags as in 00-concept-and-decision-log.md.

This section is the basis for every PROPOSED number in the specs. The speed cases are 6.5 kt (takeoff), 10 kt (upwind), 12 kt and 14.5 kt. The model's fastest point of sail inside the 10-12 kt window is a broad reach, about 22 kt of boat speed at 105 deg in 12 kt of wind (21.0 kt at 90 deg), so 14.5 kt is not the top speed; the load on that reach is the structural basis (DECIDED 29 Oct 2026; rudder-load paragraph below, section 3).

**Every figure in this section is reproduced by** r250c.py (chosen case) and sail_rescan.py (weight and sail scan, arms sized as in r250.py), re-run on 30 Oct 2026 at the rudder rev B CG (481.8 mm) with the 136 mm wetted rudder blade (first run on 28 Oct 2026; r250c.py again on 29 Oct 2026 after the structural basis moved to the top reach, which changed no performance figure) against opt.py and steel.py with the E226 foil section, the arm pivot at 84% of the Hull_v2 half-beam, the Hull_v2 waterline length (0.98 m, used only for the hull drag before takeoff), the steel-to-carbon joint and wetted arm length at 135 mm, and the rudder load from the pitch balance at each speed (opt.Cfg.rud): righting moment = (weight + that rudder downforce) x (114.6 mm pivot offset + 250 mm reach); see the Build Log.

## Method

For each boat speed the drive from the sail is compared with the drag. Drag = foil (E226 NeuralFoil section drag at the operating Reynolds number, plus induced drag CL^2/(pi.0.85.AR), AR 7.5; the foil carries the weight plus the rudder downforce at that speed) + arm strut + rudder blade + rudder T-foil + (before takeoff only) hull drag from an assumed Froude-number resistance table. Strut drag is a flat-plate friction and form-factor model calibrated to the old 18.4 N figure at 25 kt for the 36 mm arm (see the section 10 flag on this calibration), then corrected with NeuralFoil section drag for the thin sections (+22% at Reynolds number 113k, +3.5% at 195k, -2% at 290k). Sail: soft rig, aspect ratio 2.4, force polar CD = 0.06 + CL^2/(pi.AR.0.8), CL up to 1.3; the side force is limited to the righting moment / the centre-of-effort height. "Upwind" is the best boat speed over true wind angles of 50-65 deg (65 deg is the binding case: foilers sail wide upwind angles).

Rudder load (since 28 Oct 2026): the T-foil force that balances pitch at each speed, from the weight about the main foil quarter chord (all-up CG 481.8 mm from the transom, the Weight Budget layout of section 8 since 30 Oct 2026; foil at 526.4 mm; rudder stock at 20 mm, CAD), the sail drive at the centre of effort against the drag 200 mm below the waterline (assumed), and the E226 section pitching moment; the righting moment and the heel cap follow it at each speed. The structural rudder load is the top-reach figure, opt.RUD_STRUCT x weight (14.01 N), used only by the structural scripts; Cfg(pitch=False) in opt.py restores the old 175/550 performance model.

All results are for the chosen case: 2.80 kg, 0.44 m2, E226 foil, reach 250 mm, steel wet arm 22 mm x 18% wetted over 135 mm, carbon upper arm 40 mm x 20% (dry, so it adds no drag), rudder blade 34 mm x 17% wetted over 136 mm (220 mm below the hull, 84 mm of it in air at the design ride).

**The scripts that produced these numbers are in the Simulation & Analysis folder** (core VPP engine opt.py; hybrid-arm structure steel.py/steel2.py/steel3.py; the chosen-case sweep r250.py/r250b.py/r250c.py; the top-reach structural basis top_reach.py; doc-figure generator docnums.py; see that folder's README for what each one does).

## Results

| True wind | Upwind (65 deg) | Reach (90 deg) | Deep downwind (135 deg) |
|---|---|---|---|
| 8.75 kt | 13.4 kt | - | - |
| 10 kt | 15.2 kt | 18.3 kt | 11.1 kt |
| 11 kt | 16.2 kt | 19.7 kt | - |
| 12 kt | 17.3 kt | 21.0 kt | 14.7 kt |

The boat gets onto the foil from 6.25 kt of wind (takeoff speed 5.65 kt at CL 0.55, drive at least 1.10x the hull drag through the whole run), 1.25 kt of wind earlier than on the 175/550 rule (7.5 kt, takeoff 6.44 kt): the foil now carries the weight plus only the small pitch-balance downforce (about 0.4 N at takeoff) instead of 8.74 N at every speed. Crediting no T-foil lift below takeoff changes nothing. The hull drag table is assumed, and takeoff is not righting-moment-capped. The heeling force reaches the limit from about 8.75 kt of wind: at 10 kt of wind the upwind speed is 15.2 kt capped against 16.1 kt with unlimited righting moment, at 12 kt 17.3 against 20.0 kt (the cap costs more toward the top of the window, where the uncapped speed keeps rising). With the plain carbon 38 mm arm instead of the steel one, the same boat does 14.8 / 15.8 / 16.7 kt at 10 / 11 / 12 kt of wind and gets onto the foil from 6.25 kt (same 135 mm wetted length). Deep downwind at 10 kt of wind (11.1 kt) is well above takeoff speed but still the slowest point of sail, so downwind sailing is a wide reach and gybe, not a design target. The fastest point of sail is a broad reach: 18.75 kt at 100 deg in 10 kt of wind, 20.4 kt at 105 deg in 11 kt and 21.86 kt at 105 deg in 12 kt (top_reach.py, 1-deg steps; 22.15 kt on the 29 Oct run that set the structural basis). All of this section is for the thin 22 mm strut all the way to 135 mm; the straight-steel joint in Foilarm_v5 (29 Oct 2026, section 8) thickens the top 40 mm of the wet strut, adds 4.9% to the arm drag at 10 kt and costs 0.1 kt upwind at 10 kt of wind, nothing at 11 kt, 0.2 kt at 12 kt and 0.1 kt on the top reach (21.76 kt), with takeoff unchanged (the VPP steps speed in about 0.1 kt) (joint_straight.py runs r250c.py's chosen case with and without it).

## Rudder load and the structural basis

From r250c.py (re-run 30 Oct 2026 at the rudder rev B CG with the 136 mm wetted blade): downforce 0.7 / 1.4 / 2.6 / 3.9 / 5.7 N at 6.5 / 8 / 10 / 12 / 14.5 kt of boat speed, so the foil carries 28.2-33.2 N and the righting moment is 10.3-12.1 Nm over that range (the 175/550 rule gave 8.74 N, 36.2 N and 13.20 Nm everywhere); the heel cap follows, 26.0 N at 6.5 kt to 30.6 N at 14.5 kt instead of 33.4 N. The lower righting moment and the lower drag nearly cancel: upwind is 0.1 kt slower at 10-11 kt of wind and level at 12 kt. The downforce keeps growing with speed, because the sail drive acting high has to be balanced: it passes the old 8.74 N at about 17.7 kt of boat speed. On the 29 Oct 2026 run the 22.15 kt top reach (105 deg in 12 kt of wind) gave 14.0 N, 41.5 N on the foil and a static cant moment of 10.37 Nm, 13.48 Nm with the 1.30 margin (15% over the old 11.77 Nm basis); that reach load is the structural basis since 29 Oct 2026 (section 3). The 30 Oct re-run gives a slightly slower and lighter top reach (21.86 kt, 13.6 N, 41.1 N, 13.36 Nm), so the basis stands. With 30% more drag the top reach is slower (19.8 kt) but the load is higher (42.3 N), because the couple grows with the drag.

## Drag breakdown (N), E226 foil

The arm column is the 135 mm wetted steel run. Foil and T-foil drag are lower than on the 8.74 N rule (totals then 3.21 / 4.12 / 4.96 / 6.26 N) because both carry less load; the rudder downforce at each speed is in the paragraph above.

| Speed | Foil (incl. induced) | Arm | Rudder blade | Rudder T-foil | Total |
|---|---|---|---|---|---|
| 6.5 kt | 1.43 | 0.38 | 0.44 | 0.25 | 2.50 |
| 10 kt | 1.77 | 0.79 | 0.81 | 0.50 | 3.86 |
| 12 kt | 2.09 | 1.02 | 1.07 | 0.63 | 4.81 |
| 14.5 kt | 2.63 | 1.33 | 1.44 | 0.78 | 6.20 |

For comparison, the old struts (36 mm E864 arm, 30 mm E864 blade, both 38.8% thick) would add 3.71 N and 1.56 N at 10 kt, and 7.08 N and 2.98 N at 14.5 kt. E226 operating points with the pitch-balance load (NeuralFoil section, effective angle of attack): 6.5 kt Re 134k, CL 0.42, 2.1 deg, CD 0.012; 10 kt Re 206k, CL 0.19, 0.5 deg, CD 0.0093; 12 kt Re 247k, CL 0.14, about 0.1 deg, CD 0.0082; 14.5 kt Re 298k, CL 0.10, 0.0 deg, CD 0.0074 (E817: CD 0.022 / 0.019 / 0.017 / 0.014).

## Weight and sail area

With the pivot at 84% of the Hull_v2 half-beam, the E226 foil and the pitch-balance rudder load (chosen arm at every weight; sail_rescan.py, re-run on 30 Oct 2026 at the rudder rev B CG with the 136 mm wetted blade).

| All-up weight | Sail area | On the foil from | Upwind at 10 / 11 / 12 kt | Takeoff speed |
|---|---|---|---|---|
| 2.4 kg | 0.44 m2 | 5.75 kt | 14.7 / 15.6 / 16.5 kt | 5.2 kt |
| 2.8 kg | 0.40 m2 | 6.5 kt | 15.0 / 16.2 / 17.3 kt | 5.65 kt |
| **2.8 kg (chosen case)** | **0.44 m2** | 6.25 kt | 15.2 / 16.2 / 17.3 kt | 5.65 kt |
| 3.0 kg | 0.48 m2 | 6.25 kt | 15.4 / 16.5 / 17.5 kt | 5.8 kt |

Every weight and sail in the scan clears 10 kt at 10 kt of wind by more than 4 kt, and the choice is flat. At 2.8 kg, 0.40 m2 is 0.2 kt slower at 10 kt of wind and level at 11 and 12 kt, and needs 6.5 kt of wind to get onto the foil; 0.48 m2 is level at 10 and 11 kt of wind, 0.2 kt slower at 12 kt, and gets onto the foil from 6.0 kt (0.25 kt of wind earlier). Going lighter mostly buys takeoff (2.4 kg with 0.44 m2 from 5.75 kt of wind) at the cost of 0.5-0.8 kt upwind; going heavier (3.0 kg with 0.48 m2) adds about 0.2-0.3 kt upwind for a higher cant torque (the carbon upper arm's root margin falls to 3.86x at 3.0 kg, under the >=4.0x bar) and a higher takeoff speed. 2.8 kg with 0.44 m2 stays the pick; 0.48 m2 is a close alternative if an earlier takeoff matters more than 0.2 kt at 12 kt of wind. Rig mass is held inside the all-up weight by trimming ballast, and the centre-of-effort height scales with sqrt(area); both are assumptions until the rig is drawn. The CG is held at 481.8 mm in every case of the scan (an approximation: the ballast that trims the weight sits close to it).

## Sensitivity

Drag x0.7 gives 17.3 / 18.6 / 19.4 kt upwind and 5.25 kt takeoff wind; drag x1.3 gives 13.4 / 14.6 / 15.6 kt and 7.0 kt takeoff wind (30% more drag still clears 10 kt at 10 kt of wind comfortably). Centre-of-effort height x0.8 gives 15.8 / 17.2 / 18.4 kt; x1.2 gives 14.5 / 15.3 / 16.3 kt (the centre of effort now also moves the rudder load, through the sail-drive couple). The wetted lengths (135 mm arm, the full straight run, and since 30 Oct 2026 136 mm of rudder blade, the 220 mm blade less the 84 mm of air under the stern at the design ride, where 119 mm was inferred before) are the weakest input after the centre-of-effort height. 3D check of the E226 foil (foil.tools AeroBuildup, 27 Sep 2026): its drag comes out 0-6% above this model; fed back into the VPP with the 135 mm arm and the pitch-balance load (foil3d.py, re-run 30 Oct 2026) it gives upwind 15.2 / 16.2 / 17.2 kt and leaves the takeoff wind at 6.25 kt. The raised windward arm (raised_arm_vpp.py) adds up to 0.2 kt upwind at 11-12 kt of wind.

## Break-even drag

breakeven.py: r250c.py's chosen case with a multiplier on all foil-borne and pre-takeoff drag, re-run on 30 Oct 2026 at the rudder rev B CG with the 136 mm wetted blade. Drag x1.0 / 1.3 / 1.6 / 2.0 / 2.5 / 3.0: foils up from 6.25 / 7.0 / 8.0 / 9.0 / 10.25 / 11.75 kt of wind; upwind at 10 kt of wind 15.2 / 13.4 / 12.0 / 9.9 / 7.6 / 4.4 kt; upwind at 12 kt of wind 17.3 / 15.6 / 14.3 / 12.7 / 10.8 / 8.4 kt. The 8.75 kt takeoff target holds up to about x1.8, well beyond the +-30% drag uncertainty (on the 8.74 N rule it held only to about x1.3). The 10 kt upwind target now just misses at x2.0 (9.9 kt), so it holds to just under x2.0.

## Not modeled

Dynamic cant torque; the extra righting moment from the raised windward foil and arm in the headline numbers (section 3: about +0.25-0.31 Nm for a 45-64 deg raise, up to +0.2 kt upwind; left out so the numbers stay conservative); flap-deflection drag; wave and gust response; the rudder T-foil's real 3D lift; the pitch balance's assumed inputs (mast position, centre-of-effort height and drag centre: 1.8-3.4 N of downforce at 10 kt over their range, cg_layout.py).
