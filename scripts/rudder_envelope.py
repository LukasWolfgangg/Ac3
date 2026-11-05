"""Rudder blade against the heel / pitch envelope (5 Nov 2026; envelope = the builder's estimate, ASSUMPTION; numbers PROPOSED).

Envelope (builder, 5 Nov 2026): heel +-10 deg and pitch +-5 deg as gust peaks, independent of each other, symmetric because the boat tacks (the
leeward foil is the one that carries it); the sailor trims for 0-2 deg of WINDWARD heel; the flight controller holds the foil depth
but does not compensate for heel. The T-foil keeps at least 3 T-foil chords (90 mm) of water over its centre (the spec 03 rule applied to
the T-foil, kept by the builder).

Model (same as rudder_sweep.py): the leeward foil is held at its design depth (D_FOIL, 136 mm). Heel to leeward raises the centreline
rudder relative to the foil by Y_LAT x sin(heel) (364.6 mm), bow-down pitch raises it by X_AFT x sin(pitch) (506 mm); heel to windward
and bow-up pitch lower it by the same amounts. The air gap under the stern hull (84.2 mm at the design ride) follows the same way.
  1. T-foil depth over the leeward / bow-down corners, and the blade length each corner needs for 90 mm.
  2. The combined budget (and a soft guard whose only output is a bow-up bias on the pitch setpoint) Y_LAT sin(heel) + X_AFT sin(pitch) that keeps 90 mm, which a flight-controller guard can enforce.
  3. Windward heel and bow-up pitch: air gap under the stern, wetted blade, whether the thick root (dry above Z_THIN) stays dry,
     blade drag (the thick-root part with a form-factor estimate (1 + 2 t/c + 60 (t/c)^4), not NeuralFoil) and throw limit.
  4. VPP (r250c.py's chosen case) with that heel-dependent blade drag, for a steady windward heel.
Not modelled: waves and gusts, the hull-raising ride-height feedforward, ventilation of the blade, the main foil's own depth change.
Run from scripts/ like the others (needs rudder_sweep.py, r250c.py, rudder_limit.py alongside)."""
import io, contextlib, math
import numpy as np
with contextlib.redirect_stdout(io.StringIO()):
    import r250c as R
    import rudder_sweep as rs
    from rudder_limit import section, kt, rho, cla

L = 240.0                                  # blade below the hull, mm (DECIDED 4 Nov 2026)
HEEL_MAX, PITCH_MAX = 10.0, 5.0            # deg, builder's estimate
HEEL_WIND_TYP = 2.0                        # deg windward the sailor trims for (builder, 5 Nov 2026)
D_MIN = 90.0                               # mm of water over the T-foil centre: 3 T-foil chords, kept by the builder
GUARD_START, GUARD_FULL, GUARD_BIAS_MAX = 45.0, 66.0, 2.0   # soft guard (PROPOSED): mm of use where the bias starts, where it is full, and its cap in deg of bow-up
TIP_HALF = 60.0                            # mm, half the T-foil span: the shallow-side tip sits TIP_HALF x sin(heel) above the centre
MOUNT_PITCH, NEED_6_5, NEED_14_5 = 2.6, 5.0, 0.2   # deg: foil mount pitch (Foilarm_v5 / Weight Budget), E226 incidence needed at 6.5 and 14.5 kt (specs/03, foil.tools AeroBuildup 27 Sep 2026)
RAKE_POS, RAKE_NEG = 4.92, 4.96            # deg, arm rake range (Hull_v2 Servo_horn joint)
Y_LAT, X_AFT, AIR0, D_FOIL, ZT = rs.Y_LAT, rs.X_AFT, rs.AIR, rs.D_FOIL, rs.Z_THIN
sind = lambda a: math.sin(math.radians(a))

# ---- blade with the thick root: chord and t/c by depth below the hull ----
def chord(z):
    z = np.asarray(z, float)
    return np.where(z < ZT, 40.0 - 6.0 * z / ZT, 34.0 - 14.0 * (z - ZT) / (L - ZT))
def tcr(z):
    z = np.asarray(z, float)
    return np.where(z < ZT, 0.25 - 0.08 * z / ZT, 0.17)
def air_gap(heel_leeward, pitch_down):     # mm of air under the stern hull at the blade; negative = hull in the water
    return AIR0 + Y_LAT * sind(heel_leeward) + X_AFT * sind(pitch_down)
def wetted(g):
    zw = np.linspace(max(g, 0.0), L, 400); return zw, chord(zw)
def blade_drag(g, vk):                     # N, strip integral with the NeuralFoil CD0 table; thick root scaled by a form factor
    zw, c = wetted(g); V = vk * kt; q = 0.5 * rho * V * V; tc = tcr(zw)
    ff = (1 + 2 * tc + 60 * tc ** 4) / (1 + 2 * 0.17 + 60 * 0.17 ** 4)
    return float(np.trapezoid(rs.cd0(V * c / 1000 / rs.nu) * ff * q * c / 1000, zw / 1000))
def throw_limit(g, vk=22.15):              # deg at the 4x bar, wall model, x1.3 on the load, worst station (root sections included)
    zw, c = wetted(g); S = np.trapezoid(c, zw); LW = L - max(g, 0.0); V = vk * kt
    F = 0.5 * rho * V * V * (S / 1e6) * cla(2 * LW ** 2 / S) * math.radians(1); w = F * c / S
    best = 1e9
    for zq in np.concatenate([np.linspace(max(g, 0.0), ZT, 30), np.linspace(ZT, L - 1, 60)]):
        m = np.trapezoid(np.where(zw > zq, w * (zw - zq), 0.0), zw) / 1000.0
        if m <= 0: continue
        I, Zs, allow, mpm = section(float(chord(zq)), float(tcr(zq)), 4)
        best = min(best, allow / 4 * Zs / 1000.0 / (1.3 * m))
    return best

if __name__ == '__main__':
    print('ENVELOPE (builder, 5 Nov 2026): heel +-%g deg, pitch +-%g deg, independent; sailed at 0-%g deg windward; T-foil keeps %g mm of water\n' % (HEEL_MAX, PITCH_MAX, HEEL_WIND_TYP, D_MIN))

    print('1. T-foil depth below the water (mm), %g mm blade, heel to LEEWARD x pitch bow-DOWN (* = under %g mm)' % (L, D_MIN))
    print('   heel \\ pitch   ' + '  '.join('%5.0f deg' % p for p in (0, 1, 2, 3, 5)))
    for h in (0, 2, 5.7, 8, 10):
        print('   %5.1f deg     ' % h + '  '.join('%5.0f%s   ' % (rs.tfoil_depth(L, h, p), '*' if rs.tfoil_depth(L, h, p) < D_MIN else ' ') for p in (0, 1, 2, 3, 5)))
    print('   blade length for %g mm of water / level with the main foil:' % D_MIN)
    for h, p in ((5.7, 2), (HEEL_MAX, 0), (HEEL_MAX, 2), (HEEL_MAX, 3), (HEEL_MAX, PITCH_MAX)):
        print('     %4.1f deg heel, %g deg bow-down: %3.0f mm / %3.0f mm' % (h, p, rs.L_needed(D_MIN - D_FOIL, h, p), rs.L_needed(0.0, h, p)))

    budget = D_FOIL + (L - rs.Z_FOIL_REL) - D_MIN
    print('\n2. Combined budget for %g mm of water with the %g mm blade (cos terms ignored): Y_LAT sin(heel) + X_AFT sin(pitch) <= %.0f mm' % (D_MIN, L, budget))
    print('   heel %g deg uses %.0f mm, leaving %.1f deg of bow-down; pitch %g deg uses %.0f mm, leaving %.1f deg of leeward heel' % (
        HEEL_MAX, Y_LAT * sind(HEEL_MAX), math.degrees(math.asin(max(budget - Y_LAT * sind(HEEL_MAX), 0) / X_AFT)),
        PITCH_MAX, X_AFT * sind(PITCH_MAX), math.degrees(math.asin(max(budget - X_AFT * sind(PITCH_MAX), 0) / Y_LAT))))
    for Lb in (260, 280, 300):
        b = D_FOIL + (Lb - rs.Z_FOIL_REL) - D_MIN
        print('   with a %d mm blade the budget is %.0f mm (heel %g deg leaves %.1f deg of bow-down)' % (Lb, b, HEEL_MAX, math.degrees(math.asin(max(b - Y_LAT * sind(HEEL_MAX), 0) / X_AFT))))

    print('\n2b. Allowed bow-down pitch for %g mm of water, by leeward heel (%g mm blade):' % (D_MIN, L))
    for h in (0, 2, 4, 6, 8, HEEL_MAX):
        sp = ((L - rs.Z_FOIL_REL) * math.cos(math.radians(h)) + D_FOIL - D_MIN - Y_LAT * sind(h)) / X_AFT
        print('   heel %4.1f deg: %.1f deg of bow-down' % (h, math.degrees(math.asin(max(min(sp, 1.0), -1.0)))))
    print('\n2c. SOFT GUARD (PROPOSED; its only output is a bow-up bias on the pitch setpoint): use = Y_LAT sin(leeward heel) + X_AFT sin(bow-down pitch);')
    print('   no bias below %g mm, a bias rising linearly to %g deg of bow-up at %g mm, held there (the %g mm budget of section 2 is the full-bias point)' % (GUARD_START, GUARD_BIAS_MAX, GUARD_FULL, GUARD_FULL))
    print('   %-26s %10s %8s %10s %14s' % ('leeward heel / bow-down', 'use mm', 'bias', 'depth', 'depth with guard'))
    for h, p in ((5.7, 2), (8, 3), (HEEL_MAX, 2), (HEEL_MAX, 3), (HEEL_MAX, PITCH_MAX)):
        use = Y_LAT * sind(h) + X_AFT * sind(p)
        bias = GUARD_BIAS_MAX * min(max((use - GUARD_START) / (GUARD_FULL - GUARD_START), 0.0), 1.0)
        print('   %-26s %10.0f %6.1f deg %7.0f mm %11.0f mm' % ('%g deg / %g deg' % (h, p), use, bias, rs.tfoil_depth(L, h, p), rs.tfoil_depth(L, h, p - bias)))
    print('   bow-up bias lowers the stern, which is harmless on the leeward side (the air gap there is large); it is not modelled for the main foil or the pitch loop')
    print('\n3. WINDWARD heel and bow-UP pitch (foil depth held): air gap under the stern, wetted blade, thick root dry only while the gap > %.1f mm' % ZT)
    print('   %-34s %8s %8s %6s %11s %12s' % ('case', 'gap mm', 'wetted', 'root', 'drag 10 kt', 'limit 22 kt'))
    for name, hw, pu in (('level', 0, 0), ('%g deg windward (the sailing target)' % HEEL_WIND_TYP, HEEL_WIND_TYP, 0), ('%g deg windward + 1 deg bow-up' % HEEL_WIND_TYP, HEEL_WIND_TYP, 1),
                         ('5 deg windward', 5, 0), ('%g deg windward' % HEEL_MAX, HEEL_MAX, 0), ('%g deg windward + %g deg bow-up' % (HEEL_MAX, PITCH_MAX), HEEL_MAX, PITCH_MAX), ('%g deg bow-up' % PITCH_MAX, 0, PITCH_MAX)):
        g = air_gap(-hw, -pu)
        print('   %-34s %8.1f %5.0f mm %6s %9.2f N %10.2f deg' % (name, g, L - max(g, 0.0), 'dry' if g > ZT else 'WET', blade_drag(g, 10), throw_limit(g)))
    print('   thick root gets wet at %.1f deg of windward heel (level pitch); the stern hull bottom reaches the water at %.1f deg' % (
        math.degrees(math.asin((AIR0 - ZT) / Y_LAT)), math.degrees(math.asin(AIR0 / Y_LAT))))

    print('\n4. VPP (r250c.py chosen case) with the heel-dependent blade drag added, steady windward heel at every speed')
    vs = np.arange(3, 26, 1.0)
    class C3(R.C2):
        dgrid = None
        def drag(s, V, foilborne=True):
            d = R.C2.drag(s, V, foilborne)
            if s.dgrid is not None: d += float(np.interp(V / kt, s.dgrid[0], s.dgrid[1]))
            return d
    print('   %-16s %-27s %-11s %s' % ('windward heel', 'upwind 10/11/12 kt of wind', 'foils from', 'd(blade drag) 6.5 / 10 / 14.5 kt'))
    base = None
    for hw in (0, HEEL_WIND_TYP, 5, HEEL_MAX):
        g = air_gap(-hw, 0.0)
        c = C3('x', Wkg=R.W, reach=R.reach, A=R.A, tc=0.17, ac=0.038, bc=R.BC, Lw=R.Wetted / 1000, Lb=R.LB, rigAR=2.4, CD0s=0.06, es=0.80); c.hyb = R.h; c.mult = 1.0
        c.dgrid = (vs, np.array([blade_drag(g, v) - blade_drag(AIR0, v) for v in vs]))
        up = [float(c.best_upwind(t)) for t in (10, 11, 12)]; base = base or up
        print('   %-16s %-27s %-11s %s   (%s kt)' % ('%g deg' % hw, ' / '.join('%.2f' % x for x in up), c.foils_from(),
              ' / '.join('%+.2f' % (blade_drag(g, v) - blade_drag(AIR0, v)) for v in (6.5, 10, 14.5)), ' / '.join('%+.2f' % (a - b) for a, b in zip(up, base))))
    print('   (one VPP step is about 0.1 kt)')

    print('\n5. Main foil cross-check (spec 03): 120 mm clearance (3 chords of 40 mm) over the foil tip leaves 15 mm of slack, about 5.7 deg of heel or pitch;')
    print('   the envelope above is %g deg heel and %g deg pitch, so the main foil\'s own 3-chord rule is exceeded beyond 5.7 deg, in gust peaks only (builder, 5 Nov 2026; section 3 of the specs).' % (HEEL_MAX, PITCH_MAX))

    print('\n6. SHALLOW-SIDE TIP (the T-foil tip sits %g mm x sin(heel) above the centre; spec 03 states its 3-chord rule at the tip). Depth in mm, 90 mm wanted:' % TIP_HALF)
    print('   %-26s %8s %8s %12s %14s' % ('leeward heel / bow-down', 'centre', 'tip', 'tip, guard', 'L for tip 90'))
    for h, p in ((5.7, 2), (8, 3), (HEEL_MAX, 0), (HEEL_MAX, 2), (HEEL_MAX, 3), (HEEL_MAX, PITCH_MAX)):
        use = Y_LAT * sind(h) + X_AFT * sind(p)
        bias = GUARD_BIAS_MAX * min(max((use - GUARD_START) / (GUARD_FULL - GUARD_START), 0.0), 1.0)
        tip = rs.tfoil_depth(L, h, p) - TIP_HALF * sind(h); tipg = rs.tfoil_depth(L, h, p - bias) - TIP_HALF * sind(h)
        Ltip = rs.L_needed(D_MIN - D_FOIL, h, p) + TIP_HALF * sind(h) / math.cos(math.radians(h))
        print('   %-26s %8.0f %8.0f %12.0f %11.0f mm' % ('%g deg / %g deg' % (h, p), rs.tfoil_depth(L, h, p), tip, tipg, Ltip))
    print('   set against the tip the budget is (Y_LAT + %g) x sin(heel) + X_AFT x sin(pitch) <= %.0f mm: 10 deg of heel alone uses %.0f mm, so the %g mm blade' % (
        TIP_HALF, budget, (Y_LAT + TIP_HALF) * sind(HEEL_MAX), L))
    print('   cannot hold 90 mm at the tip at 10 deg of heel even level; the guard (2 deg cap) would be active at every 10 deg gust. At 7 deg heel the tip rule still leaves %.1f deg of bow-down.' % (
        math.degrees(math.asin(max(budget - (Y_LAT + TIP_HALF) * sind(7.0), 0) / X_AFT))))

    print('\n7. MAIN FOIL under the guard: a bow-up pitch bias adds the same angle of attack to the arm-mounted foil, so the arm rake has to take it out again')
    print('   rake needed = foil incidence needed - mount pitch (%.1f deg): %+.1f deg at 6.5 kt, %+.1f deg at 14.5 kt; available +%.2f / -%.2f deg' % (MOUNT_PITCH, NEED_6_5 - MOUNT_PITCH, NEED_14_5 - MOUNT_PITCH, RAKE_POS, RAKE_NEG))
    for kt_, need in ((6.5, NEED_6_5), (14.5, NEED_14_5)):
        r = need - MOUNT_PITCH
        print('   %4.1f kt: margin %+.2f deg at the nose-down end; after a %g deg bow-up bias the rake needed is %+.2f deg, margin %+.2f deg' % (kt_, RAKE_NEG + r, GUARD_BIAS_MAX, r - GUARD_BIAS_MAX, RAKE_NEG + r - GUARD_BIAS_MAX))
    print('   so a full 2 deg bias fits up to about 14.5 kt with %.1f deg to spare; above that the foil needs less incidence still (not tabulated here), so cap the bias by the rake margin left' % (RAKE_NEG + (NEED_14_5 - MOUNT_PITCH) - GUARD_BIAS_MAX))

    print('\n8. HULL-BORNE FACTOR and BEARING LOADS for the chosen blade (240 mm, 34 -> 20 mm, thick root)')
    f_fly, f_hb = throw_limit(AIR0, 6.5), throw_limit(0.0, 6.5)
    print('   throw limit at 6.5 kt: foiling %.2f deg, hull-borne (whole blade wet) %.2f deg -> hull-borne is %.0f%% of the foiling limit (uniform 220 mm blade: 79%%)' % (f_fly, f_hb, 100 * f_hb / f_fly))
    b = rs.Blade(L, 34.0, 20.0); lim, zc = b.limit(22.15); Ms, ds = b.stock_mm()
    span_b = (19.5 + 50.0) + 20.2          # mm, stern deck + 50 mm (upper bearing, ASSUMED) down to the hull bottom
    Fd = 1.3 * b.force_per_deg(22.15) * lim
    print('   at the 22.15 kt limit (%.2f deg): hull-bottom moment %.2f Nm, design side force %.1f N, bearings %.1f mm apart: upper %.0f N, lower %.0f N (uniform 220 mm blade: about 70 N and 110 N)' % (
        lim, Ms, Fd, span_b, Ms / (span_b / 1000.0), Fd + Ms / (span_b / 1000.0)))
