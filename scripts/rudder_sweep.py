"""Rudder blade depth x taper sweep (4 Nov 2026; DECISION RECORD, PROPOSED numbers only; first principles, not tested).

Question: how deep should the rudder blade go, and does a stronger taper help?  Nothing in opt.py rewards depth (drag, root
moment, rake moment and mass all rise with length), so "best depth" is defined here as a CONSTRAINT: the shallowest blade
whose T-foil stays no shallower than the main foil (by a margin) over a heel / pitch envelope.  Foil-first ventilation is the
reason: the main foil breaching limits itself, a breaching T-foil takes pitch control with it.

Model (hull frame, mm below the hull bottom, same geometry as rudder_limit.py, Hull_v2 version 19):
  * design ride: water 84.2 mm below the stern hull bottom (AIR); main foil depth 136 mm; T-foil depth L - AIR (220 mm -> 135.8).
  * blade: thick root above Z_THIN (15 mm above the design waterline, as rudder_limit.py), then a linear taper from chord CA at
    Z_THIN to CT at the T-foil (z = L); thin section, t/c fixed at 17%, n carbon plies; mass and Z from rudder_limit.section().
  * side force: lifting line on the wetted part, wall model (the higher force), AR_eff = 2 LW^2 / S; load distributed in
    proportion to local chord (same uniform-load simplification as rudder_limit.py: conservative, a real surface-piercing blade
    unloads toward the tip).  Checked at every station from Z_THIN to the tip, not only the root.
  * drag: strip integral of NeuralFoil section CD0 (E864 outline scaled to 17%, foil.tools analyze_airfoil, Ncrit 9, alpha 0) at
    the local Reynolds number.  Reported as a DELTA against the baseline blade by the same method, because the absolute strut
    drag is calibrated (CAL 0.716, retired anchor) and uncertain.
  * VPP: r250c.py's chosen case with the blade-drag delta added (Cfg subclass); opt.py and specs/ are untouched.
Not modelled: free-surface ventilation of the blade, induced drag of the blade (side force ~ 0 in steady sailing), the T-foil
junction strength, the stock and bearing redesign beyond the stock diameter below, wave and gust response.
Heel / pitch envelope is ASSUMED (nothing in the repo fixes it except the main foil's own 5.7 deg ventilation slack, specs/03).
Sign convention: pitch is taken bow DOWN as the worst case (the stern rises, the rudder tip gets shallower); heel is to leeward
(the leeward foil carries the boat, so the centreline rudder rises relative to it by 364.6 mm x sin(heel)).
"""
import io, contextlib, math
import numpy as np
with contextlib.redirect_stdout(io.StringIO()):
    from rudder_limit import section, AIR, kt, rho, cla
    from r250 import *
    from steel import Wetted
nu = 1e-6
TC, NPLY = 0.17, 4
Z_THIN = AIR - 15.0                      # thin section starts here (mm below the hull)
D_FOIL = 136.0                           # main foil depth at the design ride (rudder_limit.py docstring)
Z_FOIL_REL = 220.2                       # L at which the T-foil is exactly as deep as the main foil at the design ride (220 mm blade = 135.8 mm)
Y_LAT, X_AFT = 364.6, 506.0              # foil lateral offset from the centreline; foil-to-rudder distance (mm)
T_CH, T_SPAN = 30.0, 120.0               # S9033 T-foil
X_LEVER = (481.8 - 20.0) / 1000.0        # rudder to the all-up CG, m (Weight Budget, 30 Oct 2026)
RE = np.array([5e4, 7e4, 1e5, 1.5e5, 2e5, 3e5]); CD0 = np.array([.03229, .02541, .02003, .01624, .01412, .01182])   # NeuralFoil, E864 at 17%, alpha 0
def cd0(re): return np.interp(np.log(np.clip(re, RE[0], RE[-1])), np.log(RE), CD0)

class Blade:
    def __init__(s, L, ca, ct, n=NPLY, tc=TC):
        s.L, s.ca, s.ct, s.n, s.tc = L, ca, ct, n, tc
        s.zw = np.linspace(AIR, L, 300); s.LW = L - AIR
        s.cw = s.chord(s.zw); s.S = np.trapezoid(s.cw, s.zw)           # mm^2 wetted planform
    def chord(s, z): return s.ca + (s.ct - s.ca) * (np.asarray(z) - Z_THIN) / (s.L - Z_THIN)
    def drag(s, vk):                                                   # N, strip integral
        V = vk * kt; q = 0.5 * rho * V * V
        return float(np.trapezoid(cd0(V * s.cw / 1000 / nu) * q * s.cw / 1000, s.zw / 1000))
    def drag_moment(s, vk):                                            # N*m about the hull-bottom pivot
        V = vk * kt; q = 0.5 * rho * V * V
        return float(np.trapezoid(cd0(V * s.cw / 1000 / nu) * q * s.cw / 1000 * s.zw / 1000, s.zw / 1000))
    def force_per_deg(s, vk):                                          # N/deg, wall model
        V = vk * kt; q = 0.5 * rho * V * V
        return q * (s.S / 1e6) * cla(2 * s.LW ** 2 / s.S) * math.radians(1)
    def moment_per_deg(s, vk, zq):                                     # Nm/deg at station zq (no 1.3)
        F = s.force_per_deg(vk); w = F * s.cw / s.S                    # N per mm
        return float(np.trapezoid(w * (s.zw - zq), s.zw)) / 1000.0
    def capacity(s, zq):                                               # Nm at the 4x bar
        I, Z, allow, mpm = section(float(s.chord(zq)), s.tc, s.n)
        return allow / 4 * Z / 1000.0
    def limit(s, vk):                                                  # deg at the 4x bar, x1.3 on the load, worst station
        zs = np.linspace(Z_THIN, s.L - 1, 60)
        lim = [s.capacity(z) / (1.3 * s.moment_per_deg(vk, z)) for z in zs if s.moment_per_deg(vk, z) > 0]
        i = int(np.argmin(lim)); return min(lim), zs[i]
    def mass(s):                                                       # g, thin part only (shell), same method for every candidate
        zs = np.linspace(Z_THIN, s.L, 100)
        return float(np.trapezoid([section(float(s.chord(z)), s.tc, s.n)[3] for z in zs], zs))
    def stock_mm(s, vk=22.15):                                         # 17-4PH stock for the hull-bottom moment, 3x on yield
        M = 1.3 * s.moment_per_deg(vk, 0.0) * s.limit(vk)[0]
        return M, (32 * M * 1000 * 3.0 / (math.pi * 1000.0)) ** (1 / 3)
    def rake_moment(s, vk=22.15):                                      # Nm about the hull-bottom pivot at 5 deg of rake, x1.3
        q = 0.5 * rho * (vk * kt) ** 2
        Dt = 1.503; CMt = 0.0061 * q * T_CH * T_CH * T_SPAN / 1e9 * 1.0   # T-foil drag (topreach_checks.py) and pitching moment, as rudder_limit.py
        Mr = Dt * s.L / 1000 + s.drag_moment(vk) + 0.0061 * q * 0.0036 * 0.030
        return 1.3 * (Mr + 14.01 * s.L / 1000 * math.sin(math.radians(5)))

def depth_margin(L, heel, pitch):          # mm the T-foil sits below the main foil (negative: shallower), heel to leeward, pitch bow down (deg)
    return (L - Z_FOIL_REL) * math.cos(math.radians(heel)) - Y_LAT * math.sin(math.radians(heel)) - X_AFT * math.sin(math.radians(pitch))
def L_needed(margin, heel, pitch):
    return Z_FOIL_REL + (margin + Y_LAT * math.sin(math.radians(heel)) + X_AFT * math.sin(math.radians(pitch))) / math.cos(math.radians(heel))
def tfoil_depth(L, heel, pitch):           # mm below the water at the T-foil centre, foil held at its design depth
    return D_FOIL + depth_margin(L, heel, pitch)

if __name__ == '__main__':
    base = Blade(220.0, 34.0, 34.0)
    lim0, z0 = base.limit(22.15); M0 = base.rake_moment()
    print('BASELINE 220 mm, 34 mm constant, %d plies: wetted %.0f mm, area %.0f mm2; limit at 22.15 kt %.2f deg (rudder_limit.py thick-root figure 1.2 deg, rule %.0f/V^2), critical station %.0f mm below the hull;' % (
        NPLY, base.LW, base.S, lim0, lim0 * 22.15 ** 2, z0))
    print('  blade drag (NeuralFoil strips) 6.5 / 10 / 14.5 / 22.15 kt: %s N (model blade: 0.50 / 0.93 / 1.65 N); rake moment x1.3 at 5 deg %.2f Nm (doc 1.55); stock %.2f Nm -> %.1f mm 17-4PH; thin shell %.1f g' % (
        ' / '.join('%.2f' % base.drag(v) for v in (6.5, 10, 14.5, 22.15)), M0, *base.stock_mm(), base.mass()))

    print('\n1. DEPTH CONSTRAINT: blade length (mm below the hull) for the T-foil to stay no shallower than the main foil minus a margin')
    print('   (design ride: T-foil %.1f mm deep at L = 220; absolute depth of the T-foil centre in brackets; spec 03 puts the main foil\'s own ventilation limit at 5.7 deg)' % (220 - AIR))
    print('   %-34s %10s %10s %10s' % ('heel / pitch bow-down', 'margin 0', 'margin 15', 'margin 30'))
    for hl, pt in ((0, 0), (3, 1), (5.7, 0), (5.7, 1), (5.7, 2), (10, 2), (15, 3)):
        print('   %-34s %s' % ('%.1f deg / %.0f deg' % (hl, pt), ' '.join('%4.0f mm (%3.0f)' % (L_needed(m, hl, pt), tfoil_depth(L_needed(m, hl, pt), hl, pt) - 0) for m in (0, 15, 30))))
    print('   T-foil depth at L = 220 under the same cases:  ' + '; '.join('%.1f/%.0f deg: %.0f mm' % (hl, pt, tfoil_depth(220, hl, pt)) for hl, pt in ((0, 0), (3, 1), (5.7, 0), (5.7, 1), (5.7, 2), (10, 2))))
    print('   (3 chords of the 30 mm T-foil = 90 mm of water over its centre is the spec-03 ventilation rule applied to it)')
    print('   Second criterion, absolute: the T-foil centre keeps 3 chords (90 mm) of water over it (spec-03 rule applied to the 30 mm T-foil); blade length needed:')
    print('   %-34s %10s' % ('heel / pitch bow-down', 'L for 90 mm'))
    for hl, pt in ((0, 0), (3, 1), (5.7, 0), (5.7, 1), (5.7, 2), (10, 2), (10, 3), (15, 3)):
        print('   %-34s %7.0f mm' % ('%.1f deg / %.0f deg' % (hl, pt), L_needed(90.0 - D_FOIL, hl, pt)))

    # ---- 2. structure, drag, rake moment, mass for depth x taper -------------------------------------------------------------
    print('\n2. DEPTH x TAPER (%d-ply thin shell, t/c %.0f%%; taper = chord at the thin-section start CA -> chord at the T-foil CT)' % (NPLY, 100 * TC))
    print('   limit = blade incidence at the 4x bar at 22.15 kt (x1.3 on load); plies = plies needed to hold the baseline %.2f deg; wall check: 2 walls <= 80%% of the tip thickness' % lim0)
    print('   dD = blade drag change vs the baseline blade, same method; rake = rake moment x1.3 at 5 deg of rake (A50BHLW design load 1.55 Nm = 3.8x under its stall)')
    hdr = '   %-4s %-9s %7s %8s %7s %9s %9s %9s %7s %7s %6s %8s' % ('L', 'CA->CT', 'limit', 'plies', 'mass', 'dD 6.5kt', 'dD 10kt', 'dD 22kt', 'rake', 'stock', 'Fl/N', 'tipRe10')
    print(hdr)
    rows = []
    for L in (220, 240, 260, 280, 300):
        for ca, ct in ((34, 34), (34, 26), (34, 20), (40, 28), (40, 22), (46, 24)):
            b = Blade(L, ca, ct); lim, zc = b.limit(22.15)
            need = None
            for n in range(4, 11):
                bn = Blade(L, ca, ct, n=n)
                if bn.limit(22.15)[0] >= lim0 - 1e-6:
                    need = n; break
            wall_ok = need is not None and 2 * need * 0.2325 <= 0.8 * ct * TC
            bn = Blade(L, ca, ct, n=need or 10)
            Ms, ds = bn.stock_mm(); Flim = bn.force_per_deg(22.15) * bn.limit(22.15)[0]
            rows.append((L, ca, ct, lim, need, bn.mass(), [b.drag(v) - base.drag(v) for v in (6.5, 10, 22.15)], bn.rake_moment(), ds, Flim))
            print('   %-4d %2d->%-4d %6.2f° %6s%s %6.1fg %+8.3f %+9.3f %+9.3f %6.2f %6.1f %6.1f %8.0f' % (
                L, ca, ct, lim, need if need else '>10', ' ' if wall_ok else '!', bn.mass(), *rows[-1][6], rows[-1][7], ds, Flim, 10 * kt * ct / 1000 / nu))
    print('   (! = no feasible shell at that tip chord; limit column is for the %d-ply shell, plies column is what holding %.2f deg would take)' % (NPLY, lim0))

    # ---- 3. tip-chord sweep at fixed depth (where does taper stop paying?) ------------------------------------------------------
    print('\n3. TIP CHORD at fixed depth (4-ply shell, t/c 17%): where taper stops paying. Blade stations from the thin-section start to the tip are all checked.')
    print('   %-4s %-9s %7s %8s %9s %9s %9s %12s %12s' % ('L', 'CA->CT', 'limit', 'crit.z', 'dD 10kt', 'dD 22kt', 'mass', '1g turn 6.5kt', '1g turn 10kt'))
    F1g = 35.0 / (35.0 + 470.9) * 2.80 * 9.81
    for L, ca in ((220, 34), (260, 40)):
        for ct in (34, 30, 26, 22, 18, 14):
            b = Blade(L, ca, ct); lim, zc = b.limit(22.15)
            print('   %-4d %2d->%-4d %6.2f° %6.0fmm %+9.3f %+9.3f %7.1fg %10.2f deg %10.2f deg' % (L, ca, ct, lim, zc, b.drag(10) - base.drag(10), b.drag(22.15) - base.drag(22.15), b.mass(),
                  F1g / b.force_per_deg(6.5), F1g / b.force_per_deg(10)))
    print('   (a steady 1 g turn needs %.1f N from the rudder, so the incidence in the last two columns has to stay under the limit at that speed: limit at 6.5 kt is far above it)' % F1g)

    # ---- 4. VPP: r250c.py chosen case with the blade-drag delta added ---------------------------------------------------------------
    JOINT = 135.0; W_, reach_, A_ = 2.8, 0.25, 0.44
    h = hybrid(22, 0.18, JOINT, 40, 0.20, 6, W_, reach_, MAT['17-4PH H1025'])
    class C2(CfgH):
        mult = 1.0
        def drag(s, V, foilborne=True): return s.mult * CfgH.drag(s, V, foilborne)
    class C3(C2):
        dgrid = None
        def drag(s, V, foilborne=True):
            d = C2.drag(s, V, foilborne)
            if s.dgrid is not None: d += float(np.interp(V / kt, s.dgrid[0], s.dgrid[1]))
            return d
    def vpp(cand):
        c = C3('x', Wkg=W_, reach=reach_, A=A_, tc=0.17, ac=0.038, bc=0.034, Lw=Wetted / 1000, Lb=0.1358, rigAR=2.4, CD0s=0.06, es=0.80); c.hyb = h
        if cand is not None:
            vs = np.arange(3, 26, 1.0); c.dgrid = (vs, np.array([cand.drag(v) - base.drag(v) for v in vs]))
        return [c.best_upwind(t) for t in (10, 11, 12)], c.foils_from()
    print('\n4. VPP (r250c.py chosen case, blade drag changed by the strip-integral delta; deltas only, absolute drag is calibrated and uncertain)')
    ref = vpp(None)
    print('   %-26s upwind 10/11/12 kt of wind %-26s foils from   d(upwind)' % ('case', ''))
    print('   %-26s %-40s %5.2f' % ('baseline 220, 34 const', ' / '.join('%.2f' % x for x in ref[0]), ref[1]))
    for L, ca, ct in ((220, 34, 20), (220, 40, 22), (240, 34, 20), (240, 40, 22), (250, 40, 22), (260, 40, 22), (260, 46, 24), (280, 46, 24), (300, 46, 24), (260, 34, 34), (280, 34, 34)):
        r = vpp(Blade(L, ca, ct, n=NPLY))
        print('   %-26s %-40s %5.2f   %s' % ('%d mm, %d->%d' % (L, ca, ct), ' / '.join('%.2f' % x for x in r[0]), r[1], ' / '.join('%+.2f' % (a - b) for a, b in zip(r[0], ref[0]))))
