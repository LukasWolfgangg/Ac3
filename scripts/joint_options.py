"""29 Oct 2026: steel-to-carbon joint options at the top-reach side force (38.0 N on the 135 mm wet strut, x1.3).
Section: 22 mm chord, 18% t/c (NACA 4-digit thickness shape as a stand-in for the E864-family outline; calibrated
to the measured steel Iy 47.1 mm4). Bending about the chord line (side force). Steel 17-4PH 1000 MPa / E 197 GPa,
bar 3x on yield; carbon by the steel.py ply rule (1 x 0/90 + rest +-45: E ~21 GPa, allowable ~218 MPa), bar 4x."""
import numpy as np
F, KL, LW = 38.0, 1.3, 135.0
def M_at(z):  # N mm, side force uniform over the wet length 0..135, z = height above the foil (straight-run equivalent)
    return KL * F * (z**2 / (2*LW) if z <= LW else (z - LW/2))
x = np.linspace(0, 1, 4001)
def half_t(c, tc):   # NACA 4-digit half thickness, mm
    xx = x
    return 5*tc*c*(0.2969*np.sqrt(xx) - 0.1260*xx - 0.3516*xx**2 + 0.2843*xx**3 - 0.1036*xx**4)
Es, Ec, SY, SC = 197e3, 21.3e3, 1000.0, 218.0
def section(c, tc, tcarb, steel=True):
    h = half_t(c, tc); hs = np.clip(h - tcarb, 0, None) if steel else np.zeros_like(h)
    dx = c * (x[1]-x[0])
    Is = np.sum(2*hs**3/3)*dx; Ic = np.sum(2*(h**3 - hs**3)/3)*dx
    return Is, Ic, h.max(), hs.max()
Is0, _, h0, _ = section(22, 0.18, 0)
k = 47.114 / Is0   # shape calibration to the measured strut
def margins(M, c, tc, tcarb, steel=True):
    Is, Ic, h, hs = section(c, tc, tcarb, steel); Is *= k; Ic *= k
    EI = Es*Is + Ec*Ic
    ms = SY / (M*Es*hs/EI) if hs > 0 else 0
    mc = SC / (M*Ec*h/EI) if Ic > 0 else float('inf')
    return ms, mc
M135 = M_at(135)
print('design moment at 135 mm: %.2f Nm' % (M135/1000))
ms, _ = margins(M135, 22, 0.18, 0); print('(0) full steel section at 135 mm: steel %.1fx (bar 3x)' % ms)
_, mc = margins(M135, 22, 0.18, 9, steel=False); print('(1) all-carbon section the size of the wet strut at 135 mm: carbon %.2fx (bar 4x); a UD-rich laminate at twice the allowable: %.1fx' % (mc, 2*mc))
print('(2) steel core inside a carbon skin of constant outer profile, at 135 mm:')
for tcarb in (0.23, 0.47, 0.70, 0.93, 1.2):
    ms, mc = margins(M135, 22, 0.18, tcarb)
    print('    carbon %.2f mm per side (steel core %.2f mm thick): steel %.1fx, carbon %.1fx' % (tcarb, 22*0.18*1.0 - 2*tcarb, ms, mc))
print('(3) carbon loft alone where the steel ends (loft 22x18% at 135 mm to 40x20% at 175 mm, solid), moment at the bend height:')
for s in (10, 15, 20, 25, 30, 35):
    f = s/40; c = 22 + 18*f; tc = (22*0.18 + (8.0-3.96)*f)/c
    R = 55.0; z = 135 + R*np.sin(s/R)             # height on a 55 mm bend
    _, mc = margins(M_at(z), c, tc, 9, steel=False)
    print('    steel ends %2d mm along the bend (z %.0f mm, loft %.1f x %.1f mm): moment %.2f Nm, carbon %.1fx' % (s, z, c, c*tc, M_at(z)/1000, mc))
# masses (per arm): current spigot 4.7 g over 105-135 (about 20 mm2), sleeve 1.7 g; full section 55.7 mm2
A_full, rho_s, rho_c = 55.7, 7.85e-3, 1.55e-3
d_wet = (A_full - 4.7/(rho_s*30))*30*rho_s - 1.7
for L, Aend in ((30, 55.7), (35, 20.0), (35, 55.7)):
    V = L*(A_full + Aend)/2
    print('tang %d mm into the loft, %.0f -> %.0f mm2: steel %+.1f g, carbon displaced %.1f g; with 105-135 at full section (%+.1f g): net %+.1f g per arm' % (L, A_full, Aend, V*rho_s, V*rho_c, d_wet, V*rho_s - V*rho_c + d_wet - 4.7))
