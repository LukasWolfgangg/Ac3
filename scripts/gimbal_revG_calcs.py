"""15 Nov 2026: gimbal rev G (PROPOSED). The builder moves the arm pivot 7 mm outboard at the same height (y 114.6 -> 121.6,
z 30.6, 89% of the Hull_v2 half-beam) to shrink the hull window. The limit is the fore cross-shaft bushing (radius 9 mm)
against the hull, which narrows toward the bow (pivot study, 15 Nov 2026).

At y 121.6 the rev E/F yoke cannot fit: its outboard cheek would sit at y 137 (skin about 133.6), and the cheeks cannot
come closer than about +-12 mm because the block between them carries the 10 mm cross shaft. So the rake hinge moves
inboard of the cant axis: a stub axle along y, fixed to the cross-shaft block, carries two flanged plain bearings in a fork
housing that sits inboard of the block; the fork's bridge passes under the cross shaft to the arm socket. The rake axis
stays on the line through the pivot (along y), so the kinematics are unchanged.

Loads as gimbal_revE_calcs.py: foil load 41.48 N x 1.3 at the 250 mm reach (design), the upset adds the 38.0 N strut side
force reversed (7.7 Nm about the pivot); drag as there. Bars: 3x on yield (design), 1.5x (upset), iglidur 80 MPa static at 3x.
"""
import math
L = 41.48; LD = 1.3 * L; REACH = 250.0
SIDE_M_U = 7.7e3                     # Nmm, upset addition about the pivot
SY_TI, SY_AL = 855.0, 500.0

def bearing_loads(y_in, y_out):
    """bearings at y_in, y_out mm inboard of the pivot (both negative), lift at +REACH"""
    s = y_out - y_in
    R_out = LD * (REACH - y_in) / s
    R_in = LD * (REACH - y_out) / s
    return R_out, R_in

print('bearing loads (design) for the stub-axle hinge, bearings x mm inboard of the pivot:')
for yi, yo in ((-30, -8), (-34, -8), (-38, -8)):
    Ro, Ri = bearing_loads(yi, yo)
    for d, w in ((12, 8), (14, 8)):
        p = Ro / (d * w)
        print('  bearings at %d / %d mm (s %d): outer %.0f N, inner %.0f N; iglidur %dx%d: %.1f MPa -> %.0fx' % (yi, yo, yo - yi, Ro, Ri, d, w, p, 80 / p))
M_root = LD * (REACH + 7)            # Nmm at the stub's root on the block face, 7 mm inboard of the pivot
M_root_u = M_root / 1.3 + SIDE_M_U   # upset: static lift moment plus the reversed side force, as the rev E upset case
for lab, do, di, sy in (('Ti-6Al-4V solid 14', 14, 0, SY_TI), ('Ti-6Al-4V 14/8', 14, 8, SY_TI), ('7075 solid 14', 14, 0, SY_AL), ('7075 solid 12', 12, 0, SY_AL)):
    Z = math.pi * (do**4 - di**4) / (32 * do)
    print('stub axle %-18s Z %5.0f mm3: design %5.0f MPa %4.1fx | upset %5.0f MPa %4.1fx' % (lab, Z, M_root / Z, sy / (M_root / Z), M_root_u / Z, sy / (M_root_u / Z)))
for b, h in ((30, 6), (26, 6), (30, 5)):
    Z = b * h**2 / 6
    print('fork bridge under the shaft, Ti %dx%d mm: Z %.0f mm3, design %.0f MPa %.1fx, upset %.0f MPa %.1fx' % (b, h, Z, M_root / Z, SY_TI / (M_root / Z), M_root_u / Z, SY_TI / (M_root_u / Z)))
print('moment carried across the hull (static): %.2f Nm (4.75 Nm at y 114.6)' % (L * 121.6 / 1000))
print('righting-moment lever: %.1f mm (364.6 now), +%.1f%%' % (REACH + 121.6, 100 * 7 / 364.6))

# stub root and block (15 Nov 2026): a Ø14 stub cannot root in the rev E3 block (19 x 12 mm around the shaft); make shaft,
# block and stub one machined 7075 part with the block grown to about 22 x 22 mm, stub hollow 14/8 with a 1.5 mm root fillet
Kt = 1.5
Z = math.pi * (14**4 - 8**4) / (32 * 14)
print('\nstub 7075 14/8 at its root with Kt %.1f: design %.0f MPa %.1fx | upset %.0f MPa %.1fx' % (Kt, Kt * M_root / Z, SY_AL / (Kt * M_root / Z), Kt * M_root_u / Z, SY_AL / (Kt * M_root_u / Z)))
rho_al, rho_ti = 2.81e-3, 4.43e-3
m_stub = math.pi / 4 * (14**2 - 8**2) * 30 * rho_al
m_block = (22 * 22 - 19 * 12) * 16 * rho_al
m_housing = math.pi / 4 * (20**2 - 16**2) * 32 * rho_ti + 3.0          # Ti bearing housing, plus the horn
m_bear = 2 * 0.6
m_removed = 2 * 2.2 + 4.9 + 2 * 0.3                                     # two cheeks, rake pin, two rake bushings
dm = m_stub + m_block + m_housing + m_bear - m_removed
print('mass per side: stub %.1f g, block growth %.1f g, housing + horn %.1f g, bearings %.1f g, minus cheeks, pin, bushings %.1f g -> %+.0f g; pair %+.0f g' % (m_stub, m_block, m_housing, m_bear, m_removed, dm, 2 * dm))

# AS BUILT in Hull_v2 (15 Nov 2026, unsaved at the time of writing): stub 7075 solid d12 integral with the shaft block, from
# 11 to 26.5 mm inboard; the first layout (housing to 41 mm) broke the hull bottom from about 43 deg of cant, so the hinge
# was shortened: two iglidur 12 x 14 x 6 bearings at 12-18 and 20-26 mm inboard (centres 15 and 23 mm, 8 mm apart).
print('\nas built:')
Ro, Ri = bearing_loads(-23, -15)
p = Ro / (12 * 6)
print('  bearings at 23 / 15 mm inboard (s 8): outer %.0f N, inner %.0f N; iglidur 12x6: %.1f MPa -> %.1fx (bar 3x); upset about %.1fx' % (Ro, Ri, p, 80 / p, 80 / (p * M_root_u / (M_root / 1.0) * 1.0)))
Z12 = math.pi * 12**3 / 32
print('  stub 7075 solid d12 at its root, Kt 1.5: design %.0f MPa %.1fx | upset %.0f MPa %.1fx' % (1.5 * M_root / Z12, SY_AL / (1.5 * M_root / Z12), 1.5 * M_root_u / Z12, SY_AL / (1.5 * M_root_u / Z12)))
cad = {'cross shaft with block and stub': (36.43, 21.4), 'fork rev G (housing, bridge, horn, socket)': (36.98, 41.16), 'rake bearings / bushings': (0.72, 0.54), 'rake pin (removed)': (0.0, 4.86), 'seal bosses x2': (1.10, 4.6), 'shaft seals x2': (0.72, 1.2)}
tot = 0
for k, (new, old) in cad.items():
    tot += new - old
    print('  %-44s %6.2f g (was %6.2f g)' % (k, new, old))
print('  net per side %+.1f g, pair %+.1f g' % (tot, 2 * tot))
print('  cant sweep (CAD): nothing inside breaks the skin from -5 to 55 deg; the fork touches the hull bottom at 60 (1 mm2) and 64 deg (11 mm2); at 67 deg servo, cradle and fork hit (24-55 mm2)')
print('  window over cant -5 to 58 deg: about 803 mm2, 42.0 x 31.8 mm (today about 1140 mm2, 38 mm tall)')
print('  rake horn: Ti plate 2.6 x 4.8 mm, link 190 N at about 14 mm: %.0f MPa -> %.1fx' % (190 * 14 / (2.6 * 4.8**2 / 6), SY_TI / (190 * 14 / (2.6 * 4.8**2 / 6))))
