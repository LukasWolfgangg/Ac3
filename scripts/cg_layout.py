"""CG layout and pitch balance. Rev 30 Oct 2026 (first version 28 Oct 2026; foil section pitching moment added the same evening).
Hull frame: x mm from the transom (bow at 1000), z mm from the 2.80 kg design waterline.

Since 30 Oct 2026 the masses and positions are a snapshot of the Weight Budget (sheet Blad1, rows 21-70, read on 30 Oct 2026
after rudder rev B), so this script and the sheet give the same all-up CG. The 28 Oct version kept its own list (gimbal rev E2,
Python rake servo, an assumed 120 g rudder) and had drifted 4.8 mm from the sheet. Position sources as in the sheet:
CAD = Hull_v2 (arm, gimbal rev E3, rudder rev B), A = assumed, P = proposed layout.

What it prints: the snapshot check against the sheet; the fixed and movable masses; the pitch neutral point and static
margin; the T-foil force for pitch balance while foiling (weight about the main foil, the sail drive / drag couple and the
E226 section moment, with the drag from the VPP); the trim at rest; and what each available lever does to the CG
(battery, flight-control bay, lead, mast step), so the final mast position can be chosen with the balance in view."""
import math
G = 9.81
X_LCB, X_FOIL, Z_FOIL, X_RUD = 443.0, 526.4, -239.4, 20.0   # LCB (Design & Specs section 2); main foil quarter chord (CAD); rudder stock (CAD, Rudder_A)
TARGET = 2800.0
DRAG = {6.5: 2.43, 10: 3.86, 12: 4.87, 14.5: 6.32}          # N, total foil-borne drag from the VPP at this CG (r250c.py with opt.X_CG 0.4819
# and the 156 mm wetted, 34 -> 20 mm rudder blade, 5 Nov 2026; was 2.45/3.90/4.92/6.40 with the 136 mm blade; these are c.drag(V, True), not the r250c.py breakdown totals)
H_CE, Z_DRAG = 395.0, -200.0                                 # sail centre of effort above the waterline (Design & Specs, an assumption there too); drag centre (A)
GML = 5.0                                                    # m, longitudinal metacentric height, estimated from the Hull_v2 waterplane (A)
DECK_M = 71.0                                                # deck height at the mast (A)

# Weight Budget snapshot: (sheet row, item, nominal subtotal g, x, z, kind). kind: fixed / rig (moves with the mast step) /
# move (electronics, battery, hatches: free to place) / lead (the ballast, sized to make 2800 g)
WB = [
    (21, 'Hull shell, bottom, lower topsides and transom', 122.3, 454.3, -9.5, 'fixed'),
    (22, 'Deck and upper topsides', 131.0, 449.8, 49.6, 'fixed'),
    (23, 'Internal structure, rest (mast step, rudder mount, ballast mounts, hatch frames)', 77.0, 430.0, 10.0, 'fixed'),
    (24, 'Hatches x2 (flight-control bay + radio bay)', 50.0, 575.0, 66.0, 'move'),
    (25, 'Foil arm, upper (carbon) x2', 93.2, 531.8, -40.5, 'fixed'),
    (26, 'Steel underwater unit x2', 611.6, 520.8, -226.5, 'fixed'),
    (27, 'Gimbal rev E3 rake fork (Ti) x2', 84.4, 532.7, 21.3, 'fixed'),
    (28, 'Gimbal rev E3 cant group x2', 83.6, 527.9, 31.9, 'fixed'),
    (29, 'Arm rake servo AGF A66BHLW + horn and link x2', 133.8, 531.2, 49.8, 'fixed'),
    (30, 'Cant drive rev E3 x2', 393.0, 481.2, 29.1, 'fixed'),
    (31, 'Gimbal bushings and seals x2', 15.0, 531.4, 30.6, 'fixed'),
    (32, 'Cant motor drivers x2', 6.0, 470.0, 20.0, 'fixed'),
    (33, 'Pivot bulkheads x2', 88.4, 531.3, 23.0, 'fixed'),
    (34, 'Rudder stock, collar and cap', 29.2, 20.0, 26.0, 'fixed'),
    (35, 'Rudder blade (240 mm, 34->20 mm taper; Hull_v2 version 24)', 18.4, 14.7, -109.4, 'fixed'),
    (36, 'Rudder T-foil', 7.2, 16.0, -260.2, 'fixed'),
    (37, 'Rudder rake actuator (AGF A50BHLW, rev B)', 55.9, 69.2, 4.8, 'fixed'),
    (38, 'Rudder steering (Savox SW-0250MG, rev B)', 34.9, 55.5, 10.3, 'fixed'),
    (39, 'Rudder structure (well, carrier, cassette, deck cover, tripod)', 20.0, 24.8, 28.6, 'fixed'),
    (40, 'Mast', 153.0, None, 554.0, 'rig'),
    (41, 'Boom', 24.1, None, 131.0, 'rig'),
    (42, 'Mainsail', 27.4, None, 644.0, 'rig'),
    (43, 'Jib', 15.1, None, 454.3, 'rig'),
    (44, 'Standing rigging', 15.5, None, 646.0, 'rig'),
    (45, 'Mast rotation collar', 3.0, None, 81.0, 'rig'),
    (46, 'Traveller track and car', 15.0, 200.0, 45.0, 'fixed'),
    (47, 'Traveller servo', 13.0, 200.0, 10.0, 'fixed'),
    (48, 'Mainsheet winch motor', 10.0, 300.0, 0.0, 'fixed'),
    (49, 'Mainsheet winch pot', 4.0, 300.0, 0.0, 'fixed'),
    (50, 'Mainsheet winch driver', 3.0, 300.0, 0.0, 'fixed'),
    (51, 'Mainsheet winch drum and bracket', 12.0, 300.0, 0.0, 'fixed'),
    (52, 'Mainsheet winch control', 2.0, 300.0, 0.0, 'fixed'),
    (53, 'Mast rotation servo', 13.0, None, 0.0, 'rig'),
    (54, 'Jib track and car', 12.0, 850.0, 71.0, 'fixed'),
    (55, 'Jib sheet servo', 13.0, 800.0, -10.0, 'fixed'),
    (56, 'Rig fittings', 12.0, None, 300.0, 'rig'),
    (57, 'Microcontroller', 7.0, 500.0, -5.0, 'move'),
    (58, 'IMU', 3.0, 500.0, -5.0, 'move'),
    (59, 'Height sensor (under the hull at the pivot station)', 20.0, 531.4, -40.0, 'fixed'),
    (60, 'Flight-control battery/BEC', 35.0, 500.0, -5.0, 'move'),
    (61, 'Flight-control wiring and enclosure', 30.0, 500.0, -5.0, 'move'),
    (62, 'Flight-control mounting and potting', 15.0, 500.0, -5.0, 'move'),
    (63, 'Receiver', 14.0, 650.0, 10.0, 'move'),
    (64, 'Power switch and charge lead', 6.0, 650.0, 10.0, 'move'),
    (65, 'Servo leads', 12.0, 650.0, 10.0, 'move'),
    (66, 'Servo/RC battery (2S)', 70.0, 480.0, -25.0, 'move'),
    (67, 'Paint', 25.0, 456.0, 21.0, 'fixed'),
    (68, 'General fittings', 15.0, 450.0, 60.0, 'fixed'),
    (69, 'Fasteners', 15.0, 480.0, 15.0, 'fixed'),
    (70, 'Lead ballast, two blocks between the pivot bulkheads', None, 531.4, -30.0, 'lead'),
]
SHEET_CG = (481.932, 4.714)   # Weight Budget row 18 on 5 Nov 2026 (240 mm blade; 481.449, 4.610 before it, with the tripod rows of 30 Oct)


def rig_x(row, x_m):
    """x of the rig items for a mast step at x_m (the Weight Budget formulas: mast 1.15 m above the deck, main foot 0.48 m,
    luff 1.14 m, masthead jib tacked at x 880)"""
    return {40: x_m, 41: x_m - 240, 42: x_m - 0.42 * 480, 43: (880 + (x_m + 10) + 600) / 3, 44: (x_m + 880) / 2 if x_m < 880 else x_m,
            45: x_m, 53: x_m, 56: x_m}[row]


def items(x_m=600.0, moves=None, lead_x=None):
    """the full list for a mast step at x_m; moves = {sheet row: new x} for movable items; lead_x moves the lead"""
    moves = moves or {}
    out = []
    for row, name, m, x, z, kind in WB:
        if kind == 'rig': x = rig_x(row, x_m)
        if row in moves: x = moves[row]
        out.append([row, name, m, x, z, kind])
    lead = TARGET - sum(i[2] for i in out if i[5] != 'lead')
    for i in out:
        if i[5] == 'lead':
            i[2] = lead
            if lead_x is not None: i[3] = lead_x
    return out


def cg(lst):
    m = sum(i[2] for i in lst)
    return m, sum(i[2] * i[3] for i in lst) / m, sum(i[2] * i[4] for i in lst) / m


# E226 section pitching moment about the quarter chord (NeuralFoil via foil.tools, Ncrit 9, 28 Oct 2026): (CL, CM) per Re
CM_E226 = {133000: [(-0.090, -0.0127), (0.152, -0.0388), (0.251, -0.0473), (0.323, -0.0503), (0.417, -0.0457), (0.495, -0.0373), (0.580, -0.0297), (0.670, -0.0239)],
           206000: [(-0.020, -0.0254), (0.123, -0.0299), (0.198, -0.0333), (0.272, -0.0367), (0.399, -0.0391), (0.495, -0.0353), (0.581, -0.0294), (0.669, -0.0240)],
           298000: [(-0.005, -0.0278), (0.096, -0.0231), (0.157, -0.0234), (0.226, -0.0259), (0.368, -0.0320), (0.487, -0.0331), (0.582, -0.0298), (0.669, -0.0247)]}
RHO, NU, S_F, C_F = 999.0, 1.0e-6, 0.012, 0.040   # same water as opt.py


def cm_foil(cl, re):
    def at(tab):
        for (c0, m0), (c1, m1) in zip(tab, tab[1:]):
            if c0 <= cl <= c1: return m0 + (m1 - m0) * (cl - c0) / (c1 - c0)
        return tab[0][1] if cl < tab[0][0] else tab[-1][1]
    res = sorted(CM_E226)
    re = min(max(re, res[0]), res[-1])
    for r0, r1 in zip(res, res[1:]):
        if r0 <= re <= r1:
            return at(CM_E226[r0]) + (at(CM_E226[r1]) - at(CM_E226[r0])) * (re - r0) / (r1 - r0)


def pitch(x_cg, W=TARGET / 1000 * G, foil_moment=True):
    """rudder T-foil force (N, + = lift) for pitch balance while foiling, about the main foil quarter chord: weight at x_cg,
    rudder at X_RUD, the sail drive / drag couple (drive at H_CE, drag at Z_DRAG, bow down) and the main foil's own section
    pitching moment (E226, nose down; section CM, no 3D correction). opt.Cfg.rud does the same with the drag computed at each
    speed; this uses the VPP drag in DRAG."""
    lever = X_FOIL - X_RUD
    res = {}
    for v, D in DRAG.items():
        M_thrust = D * (H_CE - Z_DRAG) / 1000.0              # Nm, bow down
        q = 0.5 * RHO * (v * 0.5144) ** 2
        F = 0.0
        for _ in range(8):                                   # foil lift depends on the rudder force
            L = W - F
            M_foil = -cm_foil(L / (q * S_F), v * 0.5144 * C_F / NU) * q * S_F * C_F if foil_moment else 0.0   # Nm, bow down
            F = (-W * (x_cg - X_FOIL) / 1000.0 - M_thrust - M_foil) * 1000.0 / lever
        res[v] = (F, W - F)                                   # rudder lift, main foil lift
    F0 = (-W * (x_cg - X_FOIL) / 1000.0) * 1000.0 / lever
    return F0, res


def neutral_point():
    """stick-fixed pitch neutral point of main foil + rudder T-foil (lift-curve slopes from aspect ratio, no downwash:
    the leeward foil is 365 mm off the centreline, the T-foil on it)"""
    a = lambda AR: 2 * math.pi * AR / (AR + 2)
    Sf, ARf, St, ARt = 0.012, 7.5, 0.0036, 4.0
    kf, kt = Sf * a(ARf), St * a(ARt)
    return X_FOIL - (X_FOIL - X_RUD) * kt / (kf + kt)


if __name__ == '__main__':
    L0 = items()
    m, x, z = cg(L0)
    lead = [i for i in L0 if i[5] == 'lead'][0][2]
    print('Weight Budget snapshot (5 Nov 2026): %.1f g, CG x %.2f z %.2f (sheet row 18: x %.2f z %.2f); lead %.1f g (2 x %.1f g)' % (m, x, z, SHEET_CG[0], SHEET_CG[1], lead, lead / 2))
    for kind in ('fixed', 'rig', 'move', 'lead'):
        mk, xk, zk = cg([i for i in L0 if i[5] == kind])
        print('  %-5s %7.1f g at x %6.1f z %6.1f' % (kind, mk, xk, zk))
    xr = [i for i in L0 if 34 <= i[0] <= 39]
    mr, xrr, zrr = cg(xr)
    print('  rudder rows 34-39: %.1f g at x %.1f z %.1f (rev B; 135.7 g at x 31.8 for the first drawn layout, about 111 g estimated before)' % (mr, xrr, zrr))

    xnp = neutral_point()
    W = TARGET / 1000 * G
    print('\npitch neutral point (main foil + T-foil, no downwash, estimate): x %.1f mm' % xnp)
    print('static margin: CG %.1f mm ahead of it, %.1f%% of the foil-to-rudder distance (%.0f mm); CG %.1f mm aft of the main foil quarter chord' % (
        x - xnp, 100 * (x - xnp) / (X_FOIL - X_RUD), X_FOIL - X_RUD, X_FOIL - x))
    th = math.degrees((x - X_LCB) / 1000.0 / GML)
    print('trim at rest: %.2f deg bow down (GM_L %.1f m), bow/stern immersion change about %.1f mm' % (th, GML, abs(math.radians(th)) * 490))

    print('\npitch balance while foiling at this CG (rudder force + = lift; with the VPP drag at each speed):')
    F0, res = pitch(x)
    print('  weight only %+.2f N; ' % F0 + '; '.join('%.1f kt %+.2f N (foil %.1f N)' % (v, f, l) for v, (f, l) in res.items()))
    rho, S = 999.0, 0.0036
    f = res[6.5][0]; q = 0.5 * rho * (6.5 * 0.5144) ** 2
    print('  T-foil at 6.5 kt: %.2f N, section CL %.3f' % (f, abs(f) / (q * S)))
    print('  sensitivity of the 10 kt force to the assumed drag couple: ' + '; '.join(
        'CE %.0f / drag %.0f: %+.2f N' % (hce, zd, (lambda: (globals().__setitem__('H_CE', hce), globals().__setitem__('Z_DRAG', zd), pitch(x)[1][10][0])[2])())
        for hce, zd in ((300.0, -200.0), (500.0, -200.0), (395.0, -120.0), (395.0, -240.0))))
    H_CE, Z_DRAG = 395.0, -200.0

    print('\nlevers on the all-up CG (mast at x 600 unless stated; the lead is resized to keep 2800 g only if a fixed item moves):')
    def show(tag, lst):
        mm, xx, zz = cg(lst)
        print('  %-62s CG x %6.1f (%+5.1f mm), z %5.1f; static margin %4.1f%%' % (tag, xx, xx - x, zz, 100 * (xx - xnp) / (X_FOIL - X_RUD)))
        return xx
    show('as is (Weight Budget)', L0)
    show('2S battery to the radio bay (x 650)', items(moves={66: 650.0}))
    show('flight-control bay forward to x 560 (sonar stays at the pivot)', items(moves={57: 560.0, 58: 560.0, 60: 560.0, 61: 560.0, 62: 560.0}))
    show('battery to x 650 and the flight-control bay to x 560', items(moves={66: 650.0, 57: 560.0, 58: 560.0, 60: 560.0, 61: 560.0, 62: 560.0}))
    for lx in (600.0, 650.0):
        show('lead forward to x %.0f' % lx, items(lead_x=lx))
    for xm in (550.0, 650.0, 700.0):
        show('mast step at x %.0f' % xm, items(x_m=xm))
    show('mast at x 650 and the battery at x 650', items(x_m=650.0, moves={66: 650.0}))
    # the 28 Oct design point
    need = (TARGET * 490.7 - (m * x - lead * 531.4)) / lead
    print('  to put the CG back at x 490.7 with the lead alone, the lead would have to sit at x %.0f' % need)
    print('  the lead now moves the CG %.1f mm per 100 mm (it was 10.6 mm with 296 g on 28 Oct)' % (lead * 100 / TARGET))
