"""15 Nov 2026: removable rudder blade (PROPOSED, modularity). Blade, solid tip plug and T-foil come off the steel stock as
one unit at the dry blade root. A Ti socket tube (6 / 7.5 mm, 17.5 mm long, closed end) is bonded into the root in place of
the bonded stock; the stock tip has a 0.5 mm flat from 4 mm above its end; a cone-point M2.5 Ti set screw through the root
side and the tube wall locks the flat (torque and axial pull). Hull_v2, Rudder_blade.

Loads: the stock moment at the root of the blade at the PROPOSED throw limit (rudder_limit.py: 6.17 Nm at the hull bottom,
less the blade side force over the 9.5 mm to the blade top, taken as 5.9 Nm; the 6 mm stock is 3.4x there); the T-foil's
structural downforce 14.01 N x 1.3 pulls the blade off the stock. Bars: 3x on yield for metal.
"""
import math
M_ROOT, PULL = 5.9e3, 14.01 * 1.3          # Nmm, N
L = 16.5                                    # stock length in the socket, mm
SY_ST, SY_TI = 1000.0, 855.0
Z_stock = math.pi * 6**3 / 32
Z_tube = math.pi * (7.5**4 - 6**4) / (32 * 7.5)

def frac(a):
    """share of the mouth moment left in the stock at a fraction a of the socket depth (rigid stock, linear contact)"""
    t = 1 - a
    return 12 * (t**3 / 3 + (a - 0.5) * t**2 / 2)

F = M_ROOT / (2 / 3 * L)
print('contact couple %.0f N, bearing on the stock %.0f MPa (6 x 5.5 mm)' % (F, F / (6 * 5.5)))
print('stock at the socket mouth: %.0f MPa, %.1fx (unchanged from the bonded stock)' % (M_ROOT / Z_stock, SY_ST / (M_ROOT / Z_stock)))
for z_top, a in ((-19.0, 9.5 / L), (-22.0, 12.5 / L)):
    m = frac(a) * M_ROOT
    s = m / (Z_stock * 0.85) * 1.5         # flat: about 15% off the section modulus, Kt 1.5 at its end
    print('flat starting at z %.0f (%.0f%% of the depth): %.0f%% of the moment, %.0f MPa, %.1fx' % (z_top, 100 * a, 100 * frac(a), s, SY_ST / s))
print('tube alone carrying the mouth moment: %.0f MPa, %.1fx (the bonded blade root shares it)' % (M_ROOT / Z_tube, SY_TI / (M_ROOT / Z_tube)))
print('set screw: %.0f N axial pull on a cone point in a dimple, shear %.0f MPa on the M2.5 minor diameter' % (PULL, PULL / (math.pi * 2.0**2 / 4)))
print('CAD (Hull_v2): tube 1.25 g, set screw 0.05 g, blade -0.51 g for the bore: about +0.8 g')
