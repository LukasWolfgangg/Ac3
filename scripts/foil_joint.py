"""15 Nov 2026: removable main-foil horizontal (PROPOSED, modularity). Sizes two layouts of a strut / fuselage / wing joint
and prices them in strength, drag, mass and VPP speed. Nothing here is DECIDED.

Geometry from Hull_v2 (Steel_unit_v5, measured 15 Nov 2026): wing E226 40 mm chord, 4.08 mm thick at y 10 from its leading
edge; strut 22 mm x 18% with its leading edge 2 mm behind the wing's, 3.5-4.0 mm thick only from 4 to 10 mm behind its own
leading edge (0.8 mm at 0.2 mm, 1.3 mm at 18 mm), so neither layout taps a thread into the strut end.

A  collar: the strut stays the DECIDED constant profile (29 Oct 2026). A fuselage with a streamlined collar takes the strut
   end in a profile socket; two spanwise pins (d1.6 Ti screws) through the strut's thickest station, stacked vertically.
   The wing slides spanwise through a profiled tunnel in the fuselage (wire EDM) and is located by one vertical M1.6 screw.
B  foot: the strut ends in an 8 mm wide foot (changes the 29 Oct constant-profile decision, so it is Lukas's call). Two
   vertical M2 screws from under the fuselage pass through the wing and thread into the foot.
In both, the wing + fuselage is the swappable module; the strut end is the one standard interface.

CAD result (Hull_v2 version 30, layout B, the builder's choice): fuselage 11.5 g (Ti-6Al-4V), foot +3.0 g, two M2 Ti screws
0.4 g, +14.9 g per side; the fuselage clears the hull by 327 mm or more over cant -5 to 67 deg and rake +-5 deg.

Loads (specs/03 and 08): lift 41.48 N x 1.3 (top reach); foil pitching moment 0.78 Nm x 1.3; roll moment, the Foilarm_v4
tang cases (specs/08: 0.47 / 0.77 / 1.77 Nm incl. x1.3; the last is the whole half-span lost). Bars: 3x on yield (metal).
"""
import io, contextlib, numpy as np
with contextlib.redirect_stdout(io.StringIO()):
    import r250c as R
from opt import CAL, cf, form, cd0f, rho, nu, kt

L_DES, M_PITCH = 41.48 * 1.3, 0.78 * 1.3
M_ROLL = {'10 mm CP shift': 0.47, 'outer quarter lost': 0.77, 'half-span lost': 1.77}
Y_TI, Y_SS, Y_174 = 830., 450., 1000.        # MPa: Ti-6Al-4V screw, A4-70 proof, 17-4PH H1025 (typical)
A_M2, A_M16 = 2.07, 1.27                      # mm2 tensile stress areas
rho_st, rho_ti = 7.80e-3, 4.43e-3             # g/mm3

D, LF = 10.0, 64.0                            # fuselage diameter and length, mm (12 mm ahead of the wing LE, 12 aft of its TE)
def vol_fus(): return np.pi / 4 * D**2 * LF * 0.78           # cylinder with ellipsoidal nose and tail (about 0.78 of the cylinder)
TUNNEL = 40 * 4.08 * D * 0.68                 # wing inside the fuselage
SW_FUS = np.pi * D * LF * 0.80                # wetted area, mm2
def drag(V, opt):
    """net added drag (N) of one fuselage at boat speed V (m/s): body minus the wing and strut area it covers"""
    q = 0.5 * rho * V * V
    ld = LF / D; ff = 1 + 1.5 / ld**1.5 + 7 / ld**3                     # body-of-revolution form factor (Hoerner)
    body = CAL * cf(V * LF / 1000 / nu) * ff * q * SW_FUS * 1e-6       # same calibration as the struts in opt.py
    wing = cd0f(V * 0.040 / nu) * q * (40 * D) * 1e-6                  # wing planform hidden in the fuselage
    s_cov = 2 * 22 * 3.2                                               # strut sides inside the fuselage (z 2 to 5.2)
    extra = 0.0
    if opt == 'A':                                                     # collar 28 x 7 mm, 8 mm tall, over the strut
        s_cov += 2 * 22 * 8
        extra = CAL * cf(V * 0.028 / nu) * form(0.25) * q * (2 * 28 * 8) * 1e-6
    strut = CAL * cf(V * 0.022 / nu) * form(0.18) * q * s_cov * 1e-6
    return body + extra - wing - strut

def mass(opt, rho_f):
    v = vol_fus() - TUNNEL
    if opt == 'A':
        v += 0.68 * (28 * 7 - 22 * 4) * 8 - 0.68 * 22 * 4 * 3.2        # collar, less the strut socket in the fuselage
        return v * rho_f + 0.1, 0.0                                    # + 3 small Ti screws; no change to the steel unit
    v -= 0.68 * 24 * 8 * 3.2                                           # foot socket
    return v * rho_f + 0.1, 0.68 * 24 * (8 - 4) * 6 * rho_st           # foot added to the steel unit

print(__doc__.split('\n')[0])
print('\nstrength (both layouts)')
Z_WING = 1.72 / 36e6 * 1e9                    # mm3: specs/03 gives 36 MPa at the design load; centre moment 26.95 N x 63.7 mm
s = 1.72e3 / Z_WING * 3.0                     # Kt 3 at a d1.7-2.2 hole at the thickest station
print('  wing at a screw hole on the centreline: %.0f MPa (Kt 3) -> %.1fx on 17-4PH yield' % (s, Y_174 / s))
print('  lift into the strut end: bearing, %.1f MPa over the 60 mm2 strut section' % (L_DES / 60))
# A: pins in double shear for the pitch moment, socket bearing for roll, strut bending at the pin hole
dz = 5.5
Fp = M_PITCH / (dz / 1000)
print('A pins d1.6 Ti, 5.5 mm apart vertically: pitch %.0f N per pin, double shear %.0f MPa -> %.1fx (shear 0.6 x yield)' %
      (Fp, Fp / (2 * 2.01), 0.6 * Y_TI / (Fp / (2 * 2.01))))
print('  bearing on the strut (3.96 x 1.6 mm): %.0f MPa -> %.0fx' % (Fp / (3.96 * 1.6), 1.5 * Y_174 / (Fp / (3.96 * 1.6))))
for k, m in M_ROLL.items():
    F = m / (2 / 3 * 11.2e-3)                                          # socket 11.2 mm deep
    sc = F * (11.2 / 3) / (28 * 1.5**2 / 6)                            # 1.5 mm collar cheek as a cantilever
    zs = 0.11 * 22 * 4**2 * 0.85                                       # strut weak-axis modulus, less the pin hole
    ss = m * 1e3 / zs * 2.5
    print('  roll %-18s %.2f Nm: socket bearing %.0f N, collar cheek %.0f MPa (Ti %.1fx), strut at the pin hole %.0f MPa (%.1fx)' %
          (k, m, F, sc, Y_TI / sc, ss, Y_174 / ss))
# B: two M2 in the foot
for k, m in M_ROLL.items():
    T = m / 0.004 / 2
    print('B roll %-18s %.2f Nm: %.0f N per M2 (foot edge 4 mm from the bolt line) -> A4-70 %.1fx, Ti %.1fx' %
          (k, m, T, Y_SS * A_M2 / T, Y_TI * A_M2 / T))
print('  pitch: %.0f N per M2 (10 mm apart along the chord)' % (M_PITCH / 0.010))

print('\nmass per side (g) and lead left (120 g today, both arms)')
for o in 'AB':
    for nm, rf in (('17-4PH', rho_st), ('Ti-6Al-4V', rho_ti)):
        mf, ms = mass(o, rf)
        print('  %s fuselage in %-9s %.1f g + steel unit %+.1f g -> lead %.0f g' % (o, nm, mf, ms, 120 - 2 * (mf + ms)))

print('\nnet drag of one fuselage (N) at 6.5 / 10 / 12 / 14.5 kt')
for o in 'AB':
    print('  %s: %s' % (o, ' / '.join('%.3f' % drag(v * kt, o) for v in (6.5, 10, 12, 14.5))))

class F(R.C2):
    opt = None
    def drag(s, V, foilborne=True):
        d = R.C2.drag(s, V, foilborne)
        return d + (drag(V, s.opt) if (foilborne and s.opt) else 0.0)
print('\nVPP (r250c chosen case; the fuselage only on the foil in the water; interference drag not modelled)')
for o in (None, 'A', 'B'):
    c = F('x', Wkg=R.W, reach=R.reach, A=R.A, tc=0.17, ac=0.038, bc=R.BC, Lw=R.Wetted / 1000, Lb=R.LB, rigAR=2.4, CD0s=0.06, es=0.80)
    c.hyb = R.h; c.opt = o
    print('  %-5s foils from %.2f kt of wind, takeoff %.2f kt, upwind 10/11/12 %s, reach 12 kt TWA 105 %.2f' %
          (o or 'today', c.foils_from(), c.takeoff_V() / kt, [round(float(c.best_upwind(t)), 2) for t in (10, 11, 12)], c.speed(12, 105)))
