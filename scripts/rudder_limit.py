"""Rudder throw limit at speed (29 Oct 2026, PROPOSED numbers; first principles, not tested).

What limits the rudder angle at speed is the blade's own side force (Mechanisms - Rudder section 2b): the T-foil's downforce
is axial in the blade. This script finds, at each boat speed, the blade incidence at which the blade root reaches its 4x bar
(carbon, ply-weighted allowable) with the x1.3 design margin, for concept A (whole-rudder rake about a pivot at the hull
bottom, lower stock bearing there, upper bearing at the top of the stern struts).

Geometry from Hull_v2 version 19 (measured 29 Oct 2026): rudder stock at x = 20 mm from the transom, hull bottom on the
centreline z = -20.2 mm, stern deck z = +19.5 mm (hull frame, mm above the design waterline). At the design ride the water
surface is at z = 30.6 - 270 + 135 = -104.4 mm (cant axis 30.6 mm above the DWL, 270 mm arm drop, 135 mm wetted strut), so the
stern's hull bottom flies 84.2 mm above the water at level trim. Blade below the hull bottom: 220 mm put the T-foil at
the main foil's depth (136 mm) at the design ride; Design & Specs had guessed 230-260 mm before the hull was measured.
5 Nov 2026: Lukas chose 240 mm (4 Nov 2026) with the thin section tapering from 34 mm at 69 mm below the hull to 20 mm at the
T-foil (rudder_sweep.py), drawn in Hull_v2 version 24. L is now 240 mm: T-foil 155.8 mm under water at the design ride, 20 mm
below the main foil. The blocks that use one chord (c) are now the UNIFORM 34 mm comparison at that depth; the chosen tapered
blade is the last block, computed with rudder_sweep.Blade (same section() and loads).

Side force: lifting-line slope CL_alpha = 2*pi*AR/(AR+2) on the wetted part, with the free surface either a pressure-release
plane (AR = wetted span / chord) or a wall (AR doubled); the wall case is the higher force and sets the limit. Free-surface
ventilation of a surface-piercing blade is not modelled; it argues for small angles at speed anyway.
Section: the blade's E864-family outline scaled to its t/c, hollow with a wall of n plies x 0.2325 mm, inner surface offset
vertically (conservative near the nose and tail); the thin-wall formula used before (struts2.sect_t) overstates the
through-thickness second moment of this thick-walled 5.8 mm section by about half.
"""
import numpy as np, math

kt = 0.514444; rho = 999.0
up = np.array([[1,0.00778],[0.99842,0.00835],[0.99369,0.01003],[0.98583,0.01271],[0.97491,0.01628],[0.96098,0.02065],[0.94414,0.02581],[0.92449,0.03181],[0.90216,0.03871],[0.87728,0.04656],[0.85002,0.05530],[0.82055,0.06482],[0.78905,0.07492],[0.75572,0.08540],[0.72077,0.09615],[0.68443,0.10712],[0.64693,0.11835],[0.60849,0.12986],[0.56937,0.14161],[0.52981,0.15336],[0.49006,0.16467],[0.45037,0.17495],[0.41100,0.18353],[0.37219,0.18980],[0.33419,0.19335],[0.29724,0.19397],[0.26157,0.19169],[0.22740,0.18670],[0.19496,0.17926],[0.16445,0.16960],[0.13606,0.15793],[0.10997,0.14442],[0.08635,0.12929],[0.06534,0.11281],[0.04708,0.09541],[0.03168,0.07754],[0.01925,0.05967],[0.00985,0.04213],[0.00355,0.02505],[0.00040,0.00831],[0,0]])
TRAPZ = getattr(np, 'trapezoid', None) or np.trapz

def section(c, tc, n, ply=0.2325, n090=1):
    """through-thickness second moment (mm^4), section modulus (mm^3), allowable (MPa), mass per mm (g/mm) of a hollow blade"""
    x = up[::-1, 0] * c; yo = up[::-1, 1] * c * tc / 0.388
    xs = np.linspace(0, c, 4001); yo = np.interp(xs, x, yo)
    yi = np.clip(yo - n * ply, 0, None) if n else 0 * yo
    I = TRAPZ(2.0 / 3.0 * (yo ** 3 - yi ** 3), xs)
    A = TRAPZ(2 * (yo - yi), xs)
    allow = (373.0 * n090 + 186.5 * (n - n090)) / n if n else (373.0 + 3 * 186.5) / 4   # solid: same ply mix as the 4-ply shell
    return I, I / yo.max(), allow, A * 1.55e-3

Z_BOTTOM, Z_DECK = -20.2, 19.5
Z_WATER = 30.6 - 270.0 + 135.0
AIR = Z_BOTTOM - Z_WATER                     # 84.2 mm of air under the stern at the design ride
L = 240.0                                    # blade below the hull bottom (DECIDED 4 Nov 2026; was 220)
LW = L - AIR                                 # wetted length foiling
Z_UPPER = Z_DECK + 50.0                      # upper bearing at the top of the stern struts, ASSUMED 50 mm above the deck
SPAN_B = Z_UPPER - Z_BOTTOM                  # bearing spacing
X_LEVER = (490.9 - 20.0) / 1000.0            # rudder to the all-up CG, m (Weight Budget)
IZ = 0.12                                    # yaw inertia, kg m^2: 0.038 from the fore-aft spread (Weight Budget) + ~0.08 lateral, both arms down (ESTIMATE)
M_KG = 2.80
DMAX_MECH = 25.0                             # mechanical throw, deg (ASSUMED, typical)
c, tc, n = 34.0, 0.17, 4                     # thin section at its start; the blade tapers to C_TIP at the T-foil (last block)
C_TIP = 20.0

def cla(AR): return 2 * math.pi * AR / (AR + 2)
def per_deg(V, wet, chord=c):
    q = 0.5 * rho * (V * kt) ** 2
    return [q * chord / 1000 * wet / 1000 * cla(k * wet / chord) * math.radians(1) for k in (1, 2)]   # N/deg: pressure-release, wall

I, Z, allow, mpm = section(c, tc, n)
M4 = allow / 4 * Z / 1000.0                  # Nm the root may carry at the 4x bar (design value, x1.3 already in the load)
print('geometry: stern hull bottom %.1f mm above the water at the design ride; blade %.0f mm below the hull, %.0f mm wetted foiling; bearings %.0f mm apart' % (AIR, L, LW, SPAN_B))
print('uniform blade %g mm, %g%%, %d plies: I %.0f mm4 (thin-wall formula 264), Z %.1f mm3, allowable %.0f MPa, root may carry %.2f Nm at 4x; %.1f g for %g mm' % (c, 100 * tc, n, I, Z, allow, M4, mpm * L, L))

def limit(V, hullborne=False, M_cap=M4, lever_off=0.0):
    wet = L if hullborne else LW
    lever = (L - wet / 2.0) - lever_off      # root (or critical section) to the wetted centroid, mm
    f_pr, f_w = per_deg(V, wet)
    m_w = 1.3 * f_w * lever / 1000.0         # Nm per degree, design
    return M_cap / m_w, f_pr, f_w, lever

print('\nthrow limit, UNIFORM %g mm blade at %g mm (blade root at 4x, wall model), the force per degree in both models:' % (c, L))
print('  %-9s %-11s %-10s %-22s %s' % ('speed', 'mode', 'limit', 'side force N/deg', 'limit capped at the %g deg mechanical throw' % DMAX_MECH))
for V, hb in ((3, True), (5, True), (6.5, True), (6.5, False), (8, False), (10, False), (12, False), (14.5, False), (18, False), (22.15, False), (25, False)):
    d, fpr, fw, lev = limit(V, hb)
    print('  %5.2f kt  %-11s %5.2f deg  %5.1f / %5.1f          %.1f deg' % (V, 'hull-borne' if hb else 'foiling', d, fpr, fw, min(d, DMAX_MECH)))
d22, fpr22, fw22, lev = limit(22.15)
K = d22 * 22.15 ** 2
print('  flight-controller rule while foiling: limit = %.0f / V^2 deg (V in kt), capped at %g deg; hull-borne it is %.0f%% of that' % (K, DMAX_MECH, 100 * limit(6.5, True)[0] / limit(6.5)[0]))
F_lim_w = M4 / (1.3 * lev / 1000.0); F_lim_pr = F_lim_w * fpr22 / fw22
print('\nside force at the limit (constant once the schedule binds): %.1f N in the wall model, %.1f N in the pressure-release model' % (F_lim_w, F_lim_pr))
print('steering authority at the limit, 22 kt: yaw moment %.1f Nm, yaw acceleration %.0f rad/s^2 with I_z %.2f kg m^2 (ESTIMATE)' % (F_lim_pr * X_LEVER, F_lim_pr * X_LEVER / IZ, IZ))
share = 35.0 / (35.0 + 470.9)
F1g = share * M_KG * 9.81
print('a steady 1 g turn needs %.1f N from the rudder (moment balance: main strut 35 mm ahead of the CG, rudder 471 mm aft, %.1f%% of the side force) = %.2f deg at 22 kt, %.2f deg at 10 kt' % (F1g, 100 * share, F1g / fpr22, F1g / per_deg(10, LW)[0]))

print('\nstock and bearings at the limit (concept A, lower bearing at the hull bottom, upper %.0f mm above it):' % SPAN_B)
M_root = M4
F_d = M_root / (lev / 1000.0)                # design side force incl. x1.3
R_up = M_root / (SPAN_B / 1000.0); R_lo = F_d + R_up
for name, sy in (('17-4PH H1025 rod', 1000.0), ('Ti-6Al-4V rod', 880.0), ('7075-T6 rod', 503.0)):
    dmin = (32 * M_root * 1000 * 3.0 / (math.pi * sy)) ** (1 / 3)
    print('  %-17s needs %.1f mm diameter at 3x on yield' % (name, dmin))
print('  bearing loads (design): upper %.0f N, lower %.0f N; a %.1f mm stock does not fit inside a %.1f mm thick blade, so the blade root has to thicken where it meets the stock' % (R_up, R_lo, (32 * M_root * 1000 * 3.0 / (math.pi * 1000.0)) ** (1 / 3), c * tc))

print('\nthickened root (the 84 mm of blade in air while foiling does not need a thin section):')
for keep in (15.0,):
    z_thin = AIR - keep                       # the thin section starts this far below the hull, so a 15 mm ride drop keeps the thick root dry
    lev_t = (L - LW / 2.0) - z_thin
    d_t = M4 / (1.3 * fw22 * lev_t / 1000.0)
    F_t = M4 / (1.3 * lev_t / 1000.0)
    M_hull = F_t * 1.3 * (L - LW / 2.0) / 1000.0
    dmin = (32 * M_hull * 1000 * 3.0 / (math.pi * 1000.0)) ** (1 / 3)
    print('  thin %g%% section from %.0f mm below the hull (%g mm above the design waterline): the critical section moves there, lever %.0f mm instead of %.0f;' % (100 * tc, z_thin, keep, lev_t, lev))
    print('  limit %.2f deg at 22 kt (rule %.0f / V^2), hull-bottom moment %.2f Nm, stock %.1f mm 17-4PH; a 40 mm x 25%% root (10 mm thick) houses a 6.5 mm stock' % (d_t, d_t * 22.15 ** 2, M_hull, dmin))

print('\nother blades at the same %g mm, thin all the way (limit at 22 kt):' % L)
for cc, tt, nn in ((34, 0.17, 6), (40, 0.17, 4), (40, 0.20, 4), (34, 0.17, 0)):
    I2, Z2, a2, mp2 = section(cc, tt, nn)
    M42 = a2 / 4 * Z2 / 1000.0
    lev2 = L - LW / 2.0
    fw2 = per_deg(22.15, LW, cc)[1]
    print('  %g mm, %g%%, %s: root %.2f Nm at 4x, limit %.2f deg, %.0f g' % (cc, 100 * tt, ('%d plies' % nn) if nn else 'solid carbon', M42, M42 / (1.3 * fw2 * lev2 / 1000.0), mp2 * L))

q22 = 0.5 * rho * (22.15 * kt) ** 2
Dt = 1.503; Db = 2.973 * LW / 119.0          # T-foil and blade drag at 22.15 kt (topreach_checks.py; blade drag scaled to the wetted length)
CMt = 0.0061 * q22 * 0.0036 * 0.030
Mr = Dt * L / 1000 + Db * (L - LW / 2) / 1000 + CMt
Mr5 = Mr + 14.01 * L / 1000 * math.sin(math.radians(5))
print('\nrake moment about the hull-bottom pivot at 22.15 kt, %g mm blade: %.2f Nm steady (%.2f at 5 deg of rake, the downforce offset), %.2f / %.2f Nm x1.3; at the strut top (%.0f mm up) that is %.0f N x1.3' % (
    L, Mr, Mr5, 1.3 * Mr, 1.3 * Mr5, SPAN_B, 1.3 * Mr5 / (SPAN_B / 1000)))

# ---- chosen blade (4 Nov 2026, DECIDED by Lukas; numbers PROPOSED): 240 mm, thick root, 34 -> 20 mm linear taper, 4 plies ----
if __name__ == '__main__':
    from rudder_sweep import Blade, Z_THIN, X_LEVER as XL
    b = Blade(L, c, C_TIP); lim, zc = b.limit(22.15)
    Ms, ds = b.stock_mm(); Fl = b.force_per_deg(22.15) * lim
    print('\nCHOSEN BLADE %g mm, %g -> %g mm, %g%%, %d plies (tapered from %.0f mm below the hull): wetted %.0f mm' % (L, c, C_TIP, 100 * tc, n, Z_THIN, b.LW))
    print('  limit %.2f deg at 22.15 kt (rule %.0f / V^2), critical station %.0f mm below the hull; side force at the limit %.1f N; thin shell %.1f g' % (lim, lim * 22.15 ** 2, zc, Fl, b.mass()))
    print('  hull-bottom moment %.2f Nm -> %.1f mm 17-4PH at 3x on yield (6 mm drawn: %.1fx); rake moment x1.3 at 5 deg %.2f Nm' % (Ms, ds, 1000.0 / (32 * Ms * 1000 / (math.pi * 6.0 ** 3)), b.rake_moment()))
    print('  steering authority at the limit, 22 kt: yaw moment %.1f Nm (wall model, the higher force); a steady 1 g turn needs %.2f deg at 22 kt and %.2f deg at 10 kt' % (Fl * XL, F1g / b.force_per_deg(22.15), F1g / b.force_per_deg(10)))
    print('  NOT recomputed for the taper: the hull-borne factor (79% of the foiling limit was for the uniform 220 mm blade) and the bearing loads')
