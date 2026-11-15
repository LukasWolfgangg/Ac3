"""15 Nov 2026: removable arm root (PROPOSED, modularity; the builder chose the Ti fitting over a side panel). Sizes a Ti
socket on the rake fork that the carbon arm root slides into, a Ti insert bonded inside the arm cavity (the present fork
tongue, now part of the arm) and the bolts through socket, carbon and insert.

Why not bolts through the present tongue: the first bolt hole sits where the tongue carries the whole root moment (Z 57.8
mm3, gimbal_revE_calcs.py): about 1.7x at the design moment against the 3x bar, and the carbon skins in bolt bearing
about 2x against 4x. Here the socket, outside the arm, carries the moment across the joint; the carbon holes are clearance
holes, so the bolts bear only on titanium.

Geometry (Hull_v2 version 30, measured 15 Nov 2026): arm root 40 mm x 7.9 mm (E864-family 20%), skins about 1.4 mm,
cavity 5.15 mm filled by the 30 mm wide tongue; the root is straight for about 24 mm from the mouth, then curves.
Loads: design cant moment 13.48 Nm, upset 21.2 Nm (bars: 3x yield / 1.5x yield for metal, 4x / 2x failure for carbon),
the same basis as gimbal_revE_calcs.py, which applies the cant moment at the tongue.
"""
import math
T_D, T_U = 13.48, 21.18
SY_TI, E_TI, E_CF = 855.0, 110e3, 60e3          # MPa (printed Ti-6Al-4V yield as in gimbal_revE_calcs.py)
BRG_TI = 1.5 * SY_TI
K_AF = 0.70                                     # airfoil-to-rectangle factor on I (thin E864-family sections)

def I_rect(b, h): return b * h**3 / 12
t_w = 1.0                                       # socket wall, mm
I_sock = K_AF * (I_rect(40 + 2 * t_w, 7.9 + 2 * t_w) - I_rect(40, 7.9))
I_cf = K_AF * (I_rect(40, 7.9) - I_rect(37.2, 5.15))
I_ins = 57.8 * 2.575                            # tongue Z 57.8 mm3 at half-thickness 2.575 mm
EI = {'socket': E_TI * I_sock, 'carbon': E_CF * I_cf, 'insert': E_TI * I_ins}
tot = sum(EI.values())
share = {k: v / tot for k, v in EI.items()}
Z_sock = I_sock / ((7.9 + 2 * t_w) / 2)

def chk(name, s, allow, bar_d, bar_u):
    print('  %-58s design %6.0f MPa %4.1fx (bar %.1fx) | upset %6.0f MPa %4.1fx (bar %.1fx)' %
          (name, s, allow / s, bar_d, s * T_U / T_D, allow / (s * T_U / T_D), bar_u))

print('stiffness shares in the clamped zone: ' + ', '.join('%s %.0f%%' % (k, 100 * v) for k, v in share.items()))
print('socket wall %.1f mm Ti: I %.0f mm4, Z %.0f mm3 (tongue 57.8)' % (t_w, I_sock, Z_sock))
for d_b, s_b in ((3.0, 14.0), (4.0, 14.0), (4.0, 18.0)):
    F = T_D * 1e3 / s_b                         # bolt couple: the moment crosses from the arm (insert + carbon) to the socket
    d_m = {3.0: 2.39, 4.0: 3.14}[d_b]           # minor diameter
    gap = 1.4                                   # carbon skin, clearance hole: the bolt spans it unsupported
    s_bend = F * gap / 2 / (math.pi * d_m**3 / 32)
    s_shear = F / (math.pi * d_m**2 / 4)
    print('\nM%d bolts %.0f mm apart along the arm: %.0f N per bolt' % (d_b, s_b, F))
    chk('bolt bending across the 1.4 mm carbon skin (Ti, minor dia)', s_bend, SY_TI, 3, 1.5)
    chk('bolt shear (Ti, minor dia, shear 0.6 x yield)', s_shear, 0.6 * SY_TI, 3, 1.5)
    chk('bearing on the 1.0 mm socket wall', F / (t_w * d_b), BRG_TI, 3, 1.5)
    chk('bearing on the 5.15 mm insert', F / (5.15 * d_b), BRG_TI, 3, 1.5)
    kt = 2.5
    chk('socket at the bolt hole (net, Kt %.1f)' % kt, T_D * 1e3 / (Z_sock * (1 - d_b / 42)) * kt, SY_TI, 3, 1.5)
    chk('insert at its hole (its stiffness share, Kt %.1f)' % kt, share['insert'] * T_D * 1e3 / (57.8 * 0.85) * kt, SY_TI, 3, 1.5)
m_sock = (2 * (42 + 9.9) * t_w) * 26 * 4.43e-3
print('\nmass: socket about %.1f g (26 mm long); the insert is the present tongue (about 7.6 g) moved to the arm; two Ti bolts about 1 g' % m_sock)
