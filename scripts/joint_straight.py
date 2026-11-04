"""29 Oct 2026: the builder's steel-to-carbon joint (DECIDED layout): the steel strut keeps its full 22 mm x 18% section STRAIGHT up to
135 mm (no spigot, no taper: a CNC part) and the carbon is a sleeve on its OUTSIDE, tapering from a 0.1 mm feather edge at 95 mm to
T135 a side at 135 mm, where the steel ends; it closes in a short C1 wedge behind the steel's trailing edge. Foilarm_v5 version 10 is
built from outer_half() below (the same points). Checks on the top-reach strut side force (38.0 N x1.3, uniform on the 135 mm wet
length, as joint_shearlag.py): the glue line (shear-lag), the steel inside the sleeve, the all-carbon section just above the steel's end,
the CAD-measured sections along the bend, then the drag of the thicker strut top and its effect on r250c.py's chosen case.
Run from the Simulation & Analysis folder (it imports steel2.py and r250c.py for the drag and speed part)."""
# ---- geometry (shared with the Foilarm_v5 CAD build script)
"""Shared geometry for the straight-steel joint: the Foilarm_v5 steel strut profile (fit points of the CAD sketch
Loft_section_135_steel, 22 mm chord, 18%, LE at Y 2, TE at Y 24; X = thickness direction) and the carbon outer shape."""
import numpy as np
STEEL = np.array([(0.079,24.000),(0.085,23.965),(0.102,23.861),(0.130,23.688),(0.166,23.448),(0.211,23.142),(0.263,22.771),(0.325,22.339),(0.395,21.848),(0.475,21.300),(0.564,20.700),(0.662,20.052),(0.765,19.359),(0.872,18.626),(0.981,17.857),(1.093,17.057),(1.208,16.232),(1.325,15.387),(1.445,14.526),(1.565,13.656),(1.681,12.781),(1.786,11.908),(1.873,11.042),(1.937,10.188),(1.973,9.352),(1.980,8.539),(1.956,7.755),(1.905,7.003),(1.830,6.289),(1.731,5.618),(1.612,4.993),(1.474,4.419),(1.320,3.900),(1.151,3.438),(0.974,3.036),(0.791,2.697),(0.609,2.424),(0.430,2.217),(0.256,2.078),(0.085,2.009),(0.000,2.000)])
CAV = np.array([(0.151,17.082),(0.172,16.933),(0.209,16.662),(0.256,16.325),(0.309,15.945),(0.367,15.528),(0.429,15.083),(0.494,14.614),(0.562,14.125),(0.631,13.621),(0.701,13.102),(0.770,12.572),(0.835,12.033),(0.894,11.487),(0.947,10.935),(0.990,10.380),(1.022,9.824),(1.042,9.267),(1.047,8.713),(1.038,8.164),(1.014,7.620),(0.976,7.086),(0.923,6.562),(0.855,6.052),(0.773,5.561),(0.676,5.081),(0.565,4.645),(0.440,4.199),(0.302,3.857),(0.153,3.396),(0.000,2.930)])
def full(half):
    """closed loop from the upper-surface half (TE -> LE, X >= 0): TE upper -> LE -> TE lower"""
    lo = half[::-1][1:].copy(); lo[:, 0] *= -1
    return np.vstack([half, lo])
def _upper_dense(n=400):
    h = STEEL[::-1]          # LE -> TE, X >= 0
    s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(h, axis=0).T))])
    u = np.linspace(0, s[-1], n)
    return np.column_stack([np.interp(u, s, h[:, 0]), np.interp(u, s, h[:, 1])])
def outer_half(t, te_gap=0.15, ext_per_t=4.7, n=60):
    """upper half of the carbon outer at a local laminate thickness t (mm): the steel offset normal-outward by t, then a smooth
    (C1, cubic Hermite) closing wedge behind the steel's trailing edge, ext_per_t * t long, ending at a te_gap mm trailing edge.
    Returned TE -> LE (same order as STEEL), X >= 0."""
    h = _upper_dense()
    d = np.gradient(h, axis=0); d /= np.hypot(d[:, 0], d[:, 1])[:, None]
    nrm = np.column_stack([d[:, 1], -d[:, 0]])          # outward for LE->TE traversal of the upper (X>0) side
    nrm[0] = (-1.0, 0.0) if False else nrm[0]
    o = h + t*nrm
    o = o[o[:, 0] >= 0]                                  # drop points the LE offset pushed across the chord line
    if o[0, 0] > 1e-6: o = np.vstack([[0.0, h[0, 1] - t], o])  # LE point on the chord line
    y0, x0 = o[-1, 1], o[-1, 0]
    sl = (o[-1, 0] - o[-5, 0])/(o[-1, 1] - o[-5, 1])     # dX/dY at the end of the offset (negative)
    e = max(ext_per_t*t, 0.05)
    y1, x1 = y0 + e, te_gap/2
    s1 = 2*(x1 - x0)/e - sl                              # end slope from a parabola-like closure (keeps it monotone)
    u = np.linspace(0, 1, 12)[1:]
    h00, h10, h01, h11 = 2*u**3-3*u**2+1, u**3-2*u**2+u, -2*u**3+3*u**2, u**3-u**2
    xe = h00*x0 + h10*e*sl + h01*x1 + h11*e*s1
    o = np.vstack([o, np.column_stack([xe, y0 + u*e])])
    # resample to n points by arc length, LE -> TE, then return TE -> LE
    s = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(o, axis=0).T))])
    uu = np.linspace(0, s[-1], n)**1.0
    w = 0.5 - 0.5*np.cos(np.linspace(0, np.pi, n))      # cluster at LE and TE
    uu = w*s[-1]
    r = np.column_stack([np.interp(uu, s, o[:, 0]), np.interp(uu, s, o[:, 1])])
    r[0, 0] = 0.0
    return r[::-1]
def poly_props(P):
    """area and second moment about the chord line (X = 0) of a closed polygon (X, Y), any orientation"""
    x, y = P[:, 0], P[:, 1]; x2, y2 = np.roll(x, -1), np.roll(y, -1)
    cr = x*y2 - x2*y
    A = cr.sum()/2
    Ixx = ((x*x + x*x2 + x2*x2)*cr).sum()/12      # integral of X^2 dA (bending about the chord line, side force)
    return abs(A), abs(Ixx)

# ---- loads and materials (as joint_shearlag.py / joint_scarf.py)
F, KL, LW = 38.0, 1.3, 135.0                     # top-reach strut side force (N), x1.3, wet length (mm), uniform load
def M_at(z): return KL*F*(z**2/(2*LW) if z <= LW else (z - LW/2))   # N mm; above 135 mm the lever is the height above the load centre
Es, SY = 197e3, 1000.0                           # 17-4PH H1025
WOVEN = (21.3e3, 218.0)                          # steel.py ply rule (1 x 0/90 + 5 x +-45): E, allowable (MPa)
UD = (100e3, 600.0)                              # UD along the strut, wet-laid (ASSUMED, compression-limited; coupon to confirm)
PM45 = (15e3, 150.0)                             # one +-45 ply on the outside (ASSUMED)
Z0, Z1 = 95.0, 135.0                             # sleeve: feather edge at 95 mm, full thickness where the steel ends at 135 mm
T_FEATHER = 0.10                                 # mm at 95 mm (the glass isolation ply / last ply drop)
# ---- glue line: shear-lag model of joint_shearlag.py (equivalent rectangle 22 mm wide; 2.95 mm reproduces the steel Iy)
b, Ga, ta, T_EQ = 22.0, 1000.0, 0.2, 2.95        # G 1000 MPa over 0.2 mm (epoxy + the glass ply), ASSUMED
def glue_peak(t135, Ec, n=2001):
    z = np.linspace(Z0, Z1, n); h = z[1] - z[0]
    ts = np.full(n, T_EQ); tc = np.maximum(T_FEATHER + (t135 - T_FEATHER)*(z - Z0)/(Z1 - Z0), 0.005); d = ts/2 + tc/2
    K = Ga*b/ta
    a = K*(1/(Ec*b*tc) + 12*d/(Es*b*ts**2)); r = -K*6*np.array([M_at(v) for v in z])/(Es*b*ts**2)
    A = np.zeros((n, n)); rhs = np.zeros(n)
    A[0, 0] = 1; A[-1, -1] = 1; rhs[-1] = M_at(Z1)/(2*d[-1])
    for i in range(1, n - 1):
        A[i, i-1] = 1/h**2; A[i, i] = -2/h**2 - a[i]; A[i, i+1] = 1/h**2; rhs[i] = r[i]
    N = np.linalg.solve(A, rhs); tau = np.gradient(N, z)/b
    i = np.argmax(np.abs(tau)); return abs(tau[i]), z[i]
GLUE_WET = 15.0                                  # MPa, ASSUMED wet shear strength (strength_v4.py); bar 3x
def lam(t135, t45):
    """sleeve laminate at 135 mm: t45 of +-45 outside, the rest UD; returns (E_eff, list of (thickness, (E, allow)))"""
    tu = t135 - t45
    E = (tu*UD[0] + t45*PM45[0])/t135
    return E, tu
def section_at_135(t135, t45, inner=WOVEN):
    """all-carbon section just above the steel's end: the sleeve band (steel outline -> outer) as UD + +-45, the core inside
    the steel outline (around the arm cavity) as the arm's woven laminate. Transformed-section bending about the chord line."""
    O = full(outer_half(t135)); P = full(STEEL); C = full(CAV)
    AO, IO = poly_props(O); AP, IP = poly_props(P); AC, IC = poly_props(C)
    xO = O[:, 0].max(); xP = P[:, 0].max()
    # band split: outer t45 as +-45 (approximated as the band between offsets t135 - t45 and t135), rest UD
    Om = full(outer_half(t135 - t45)) if t45 > 0 else P
    AOm, IOm = poly_props(Om)
    parts = ([('+-45 band', IO - IOm, PM45, xO)] if t45 > 0 else []) + [('UD band', IOm - IP if t45 > 0 else IO - IP, UD, Om[:, 0].max() if t45 > 0 else xO),
             ('core (woven)', IP - IC, inner, xP)]
    EI = sum(I*m[0] for _, I, m, _ in parts)
    M = M_at(135.0)
    res = [(nm, m[1]/(M*m[0]*x/EI)) for nm, I, m, x in parts if I > 0]
    return res, AO - AC
def steel_in_sleeve():
    A, I = poly_props(full(STEEL)); return SY/(M_at(135.0)*STEEL[:, 0].max()/I)   # conservative: the steel alone at 135 mm

# ---- drag of the thickened strut top (95-135 mm is wet at the design ride), with the project's strut model (steel2.strut_c)
T135, T45 = 1.50, 0.20                           # CHOSEN: 1.3 mm UD + one 0.2 mm +-45 ply outside, per side at 135 mm
def strut_slices(t135=T135, dz=1.0):
    """(chord m, t/c, length m) slices of the wet strut 0..135 mm"""
    out = []
    for z in np.arange(dz/2, LW, dz):
        if z < Z0: out.append((0.022, 0.18, dz/1000)); continue
        t = T_FEATHER + (t135 - T_FEATHER)*(z - Z0)/(Z1 - Z0)
        o = outer_half(t); c = np.ptp(o[:, 1]); out.append((c/1000, 2*o[:, 0].max()/c, dz/1000))
    return out
# ---- carbon sections along the bend, measured in Foilarm_v5 version 10 (29 Oct 2026): planes normal to Arm_path_v3 at path
# length s (mm from 135 mm), z height above the foil line (mm), I about the section's chord axis through its centroid (mm4),
# outer-fibre distance v (mm); all-carbon (hollow, around the arm cavity) and taken as homogeneous
BEND = [(0.5, 135.50, 362.3, 3.47), (5, 139.99, 363.0, 3.45), (10, 144.92, 372.1, 3.46), (15, 149.70, 390.8, 3.52), (20, 154.24, 418.6, 3.61),
        (25, 158.48, 453.8, 3.72), (30, 162.36, 492.6, 3.84), (35, 165.88, 529.2, 3.93), (40, 169.06, 558.9, 4.00), (60, 178.97, 558.8, 4.00)]
def bend_margins(allow): return [(s, z, allow*I/(M_at(z)*v)) for s, z, I, v in BEND]
if __name__ == '__main__':
    import io, contextlib
    print('steel alone at 135 mm (conservative, the carbon shares it): %.1fx (bar 3x)' % steel_in_sleeve())
    for t135, t45 in ((1.25, 0.0), (1.25, 0.2), (1.5, 0.2), (1.5, 0.0), (1.75, 0.2)):
        E, tu = lam(t135, t45); tp, zp = glue_peak(t135, E); res, A = section_at_135(t135, t45)
        o = full(outer_half(t135))
        print('sleeve %.2f mm a side at 135 mm (UD %.2f + +-45 %.2f, E %.0f GPa): outer %.1f x %.2f mm (%.1f%%); glue peak %.1f MPa at %.0f mm -> %.1fx (bar 3x); all-carbon section just above the steel end: %s (bar 4x)' % (
            t135, tu, t45, E/1e3, np.ptp(o[:, 1]), 2*o[:, 0].max(), 100*2*o[:, 0].max()/np.ptp(o[:, 1]), tp, zp, GLUE_WET/tp, ', '.join('%s %.1fx' % r for r in res)))
    for lab, Ec in (('woven ply-rule', WOVEN[0]),):
        tp, zp = glue_peak(T135, Ec); print('same 1.5 mm sleeve in %s carbon: glue %.1fx -> FAILS' % (lab, GLUE_WET/tp))
    for lab, (E, allow) in (('woven ply rule', WOVEN), ('UD', UD)):
        print('carbon along the bend (CAD sections), %s: ' % lab + ', '.join('s %g %.1fx' % (s_, m) for s_, z, m in bend_margins(allow)) + ' (bar 4x)')
    print('steel at the M1.6 screw hole (125 mm, steel alone, Kt 2): %.1fx (bar 3x)' % (SY/(2*M_at(125.0)*STEEL[:, 0].max()/poly_props(full(STEEL))[1])))
    with contextlib.redirect_stdout(io.StringIO()):
        from steel2 import strut_c
        import r250c as R
    kt = R.kt
    def arm(V, t135=T135): return sum(strut_c(V, c, L, tc) for c, tc, L in strut_slices(t135))
    base = lambda V: strut_c(V, 0.022, 0.135, 0.18)
    for vk in (6.5, 10, 14.5, 22.15):
        V = vk*kt; print('arm strut drag at %5.2f kt: %.3f N now, %.3f N with the 1.5 mm sleeve (%+.1f%%); 1.25 mm: %+.1f%%' % (vk, base(V), arm(V), 100*(arm(V)/base(V) - 1), 100*(arm(V, 1.25)/base(V) - 1)))
    class C3(R.C2):
        dd = True
        def drag(s, V, foilborne=True):
            return R.C2.drag(s, V, foilborne) + (s.mult*(arm(V) - base(V)) if s.dd else 0.0)
    def mk(dd):
        c = C3('x', Wkg=R.W, reach=R.reach, A=R.A, tc=0.17, ac=0.038, bc=0.034, Lw=R.Wetted/1000, Lb=R.LB, rigAR=2.4, CD0s=0.06, es=0.80)   # Lb: 30 Oct 2026, the 136 mm wetted rudder blade (r250c.LB)
        c.hyb = R.h; c.dd = dd; return c
    for dd, lab in ((False, 'now (thin 22 mm strut to 135 mm)'), (True, 'straight steel + 1.5 mm UD sleeve')):
        c = mk(dd)
        print('%-36s upwind 10/11/12 kt: %.2f %.2f %.2f kt; foils from %.2f kt; top reach (12 kt, 105 deg) %.2f kt' % (lab, c.best_upwind(10), c.best_upwind(11), c.best_upwind(12), c.foils_from(), c.speed(12, 105)), flush=True)
