# Design & Specs - 12. Modularity and service

Status tags as in 00-concept-and-decision-log.md. Started 15 Nov 2026, at the builder's request: a part that breaks, or turns out to be scaled wrong, should be easy to replace. Every item here is PROPOSED unless marked otherwise. Dimensions, masses and checks of the modules live in the section each one points to; this section holds only the module list, the access rules and the fastener standard.

## 12.1 Modules

| Module | Comes off at | Fasteners | Access | Details |
|---|---|---|---|---|
| Main foil wing + its fuselage | Strut foot (layout B) | 2 x M2 countersunk Ti | Outside, from under the fuselage | Section 3, removable wing |
| Arm (carbon arm + steel strut, with the wing on) | Ti socket on the rake fork (rev F) | 2 x M4 pan-head Ti | Outside the hull side, at cant 0 | Section 3, removable arm |
| Rudder blade + tip plug + T-foil | Ti socket tube on the stock tip | 1 x M2.5 cone-point set screw | Outside, at the dry blade root | Section 4, removable blade |
| Rake servo | Servo cradle | Servo ears; rake link pin | Servo-bay service hatch | Section 3, gimbal service access |
| Cant motor, spur pair, encoder | Worm housing | Motor plate screws | Worm-bay service hatch (pinion and encoder open, see 12.4) | Section 3, gimbal service access |
| Worm + bearings | Worm housing, bolt-on top cap | Cap screws | Worm-bay hatch, cap off; the worm unscrews out of the sector (arm off) | Section 3, gimbal service access |
| Worm sector, cross shaft, cradle, fork | OPEN: no path yet | - | The shaft cannot slide out: its rake block cannot pass the 18 mm bushing hole (the rev E3 block's corners were 11.2 mm from the axis against a 9 mm hole, and rev G's 22 x 22 mm block is bigger). Options: a split block, or a bolt-on sector. The fork can come off the rev G stub axle by sliding inboard about 15 mm (the cradle is cut clear) once the arm is off | Section 3, gimbal rev G |
| Height sensor | Screwed bracket on its pocket (not drawn yet) | 2 x M3 through the A02YYUW ears | Needs a way in at x 800 (see 12.4) | Section 3, sensor placement |
| Flight-control electronics | Removable tray (not drawn yet) | Tray screws; one connector block | Flight-control bay hatch | 12.3 |
| Lead | Screwed blocks on a rail (not drawn yet) | M3 into threaded inserts | Dry bay inboard of the wet wells | 12.3 |
| Mast | Mast step track (rig not drawn yet) | Step clamp screws | Deck | 12.3 |

Not modular by choice: the steel-to-carbon joint (bonded, coupon test open; the whole arm is the spare instead), the fork's bonded insert in the arm, the pivot bulkheads (hull structure). A bolt-in gimbal cassette was found impossible (section 3, gimbal service access).

## 12.2 Spares

One spare of each, in order of likely need: wing with fuselage (both sides use the same part, mirrored if the build demands), rudder blade unit, rake servo, worm sector and worm, arm. Each spare must be made to the same interface as the part it replaces, so a rescaled wing only has to match the strut foot, the fuselage tunnel and the two screw positions.

## 12.3 Packaging rules for parts not drawn yet

- **Cross-shaft withdrawal path:** dropped 15 Nov 2026 - the shaft cannot pass the bushing holes (12.1), so this space is not needed until a split block or bolt-on sector is designed.
- **Electronics tray:** the flight controller, IMU, motor drivers and receiver sit on one removable PA12-CF tray under the flight-control bay hatch, with one connector block for every lead that leaves it (servos, cant motors, encoders, sensor, battery), so the tray unplugs and lifts out without unsoldering. The IMU stays rigidly screwed to the tray; the tray is screwed to the hull, not clipped.
- **Lead:** the lead blocks are screwed (M3) to a bonded ply rail with threaded inserts at a 10 mm pitch, so they can be re-trimmed and moved fore and aft; it is a weak lever now (Weight Budget and section 8).
- **Mast step:** a track or a row of step positions instead of one fixed step, so the mast can move fore and aft; the mast step is the strongest CG lever left (section 8). Range and pitch to be set when the rig is drawn.
- **Height sensor:** a screwed bracket using the A02YYUW's own ears, so the sensor and its 300 mm cable can be replaced from inside.

Booked in the Weight Budget on 15 Nov 2026 as estimates (not drawn): tray about 6 g and connector block about 3 g (row 61), lead rail about 3.5 g and mast step track about 6 g (row 23), sensor bracket about 2 g (row 59, replacing the 10 g pod estimate).

## 12.4 Open

- Spur pinion under the cant motor plate and the encoder board under the deck edge (section 3).
- A way in at the height sensor (x 800, in the bow): none of the drawn hatches reaches it. The builder may replace the small gimbal hatches with one large hatch over most of the rear deck; the bow access decision belongs with that.
- Hull slot for the arm socket and bolt heads, and the arm's withdrawal path through the skin (section 3, removable arm), to be drawn once the hull is shelled there.
- Fastener standard (12.5) not yet applied in CAD: the rudder set screw is still M2.5.

## 12.5 Fastener standard

Titanium (Ti-6Al-4V) screws throughout, isolated from carbon where they sit in it, threadlocker on every thread that is not meant to be undone in the field. Proposed sizes, so one small hex-key set (1.25 to 2.5 mm, depending on the head styles chosen) services the boat:

| Size | Where | Note |
|---|---|---|
| M1.6 | Steel-to-carbon joint screw | Bonded joint, not a service fastener |
| M2 | Wing to strut foot; service-hatch lids | |
| M3 | Sensor bracket; lead rail; tray; the rudder set screw (to be changed from M2.5) | (the rake-pin grub screw went with the rake pin in gimbal rev G) |
| M4 | Arm socket bolts | M3 fails there (section 3) |
