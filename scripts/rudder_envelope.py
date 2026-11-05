"""Rudder blade against the heel / pitch envelope (5 Nov 2026; envelope = the builder's estimate, ASSUMPTION; numbers PROPOSED).

Envelope (builder, 5 Nov 2026): heel +-10 deg and pitch +-5 deg, independent of each other, symmetric because the boat tacks (the
leeward foil is the one that carries it); the sailor trims for 0-2 deg of WINDWARD heel; the flight controller holds the foil depth
but does not compensate for heel. The T-foil keeps at least 3 T-foil chords (90 mm) of water over its centre (the spec 03 rule applied to
the T-foil, kept by the builder).

Model (same as rudder_sweep.py): the leeward foil is held at its design depth (D_FOIL, 136 mm). Heel to leeward raises the centreline
rudder relative to the foil by Y_LAT x sin(heel) (364.6 mm), bow-down pitch raises it by X_AFT x sin(pitch) (506 mm); heel to windward
and bow-up pitch lower it by the same amounts. The air gap under the stern hull (84.2 mm at the design ride) follows the same way.
  1. T-foil depth over the leeward / bow-down corners, and the blade length each corner needs for 90 mm.
  2. The combined budget Y_LAT sin(heel) + X_AFT sin(pitch) that keeps 90 mm, which a flight-controller guard can enforce.
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
    print('   the envelope above is %g deg heel and %g deg pitch, so the main foil\'s own 3-chord rule is broken beyond 5.7 deg (OPEN, section 3 of the specs).' % (HEEL_MAX, PITCH_MAX))
