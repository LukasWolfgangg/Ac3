"""29 Oct 2026: straight-steel (CNC, no bends) steel-to-carbon joint for the top-reach side force (38.0 N, x1.3).
The builder's layout: the carbon sleeve runs down the straight vertical strut, thickening upward while the steel spigot inside
thins, outer profile unchanged (22 mm chord, 18% t/c); the steel ends straight at 135 mm (top of the vertical run).
Section model as joint_options.py (NACA-shape stand-in calibrated to the measured steel Iy 47.1 mm4), bending about the chord line.
Carbon: (a) the steel.py ply rule (1 x 0/90 + 5 x +-45 woven: E ~21 GPa, allowable ~218 MPa), (b) unidirectional along the strut,
wet-laid: E ~100 GPa, allowable ~600 MPa (ASSUMED, compression-limited; to confirm with a coupon). Bars 3x steel yield, 4x carbon.
Bond: interface shear from the rate at which the steel's share of the moment falls, over the two faces of the core; peak x1.3
for a scarf (ASSUMED); against the assumed 15 MPa wet strength of strength_v4.py."""
import numpy as np
F, KL, LW = 38.0, 1.3, 135.0
def M_at(z): return KL*F*(z**2/(2*LW) if z <= LW else (z - LW/2))   # N mm
x = np.linspace(0, 1, 2001)
def half_t(c, tc): return 5*tc*c*(0.2969*np.sqrt(x) - 0.1260*x - 0.3516*x**2 + 0.2843*x**3 - 0.1036*x**4)
h0 = half_t(22, 0.18); dx = 22*(x[1]-x[0])
k = 47.114/(np.sum(2*h0**3/3)*dx)
Es, SY = 197e3, 1000.0
CARB = {'woven ply-rule carbon': (21.3e3, 218.0), 'UD carbon along the strut (assumed)': (100e3, 600.0)}
def sec(tcarb, h=h0):
    hs = np.clip(h - tcarb, 0, None)
    Is = k*np.sum(2*hs**3/3)*dx; Ic = k*np.sum(2*(h**3 - hs**3)/3)*dx
    return Is, Ic, h.max(), hs.max(), hs
def marg(M, tcarb, Ec, Sc):
    Is, Ic, h, hs, _ = sec(tcarb)
    EI = Es*Is + Ec*Ic
    ms = SY/(M*Es*hs/EI) if hs > 1e-9 else float('inf')
    mc = Sc/(M*Ec*h/EI) if Ic > 1e-9 else float('inf')
    return ms, mc, Es*Is/EI
tmax = h0.max()   # 1.98 mm half thickness
for name, (Ec, Sc) in CARB.items():
    print('== %s' % name)
    print('  all-carbon section at 135 mm (steel ends below the waterline): carbon %.1fx' % marg(M_at(135), 9, Ec, Sc)[1])
    for z0, t_end in ((95, 1.98), (105, 1.98), (95, 1.7)):
        # skin grows linearly from 0 at z0 to t_end per side at 135 mm; steel core = profile minus skin
        worst = (9e9, None); share = []
        zs = np.linspace(z0, 135, 41)
        for z in zs:
            t = t_end*(z - z0)/(135 - z0)
            ms, mc, fs = marg(M_at(z), t, Ec, Sc)
            share.append(fs*M_at(z))
            for m, lab in ((ms/3, 'steel'), (mc/4, 'carbon')):
                if m < worst[0]: worst = (m, (lab, z, t, ms if lab == 'steel' else mc))
        # bond shear: change along the strut of the axial force in the tension-side skin, over the face width
        def Nskin(z, t):
            Is, Ic, h, hs, hsv = sec(t); EI = Es*Is + Ec*Ic
            # skin strip at each chord station spans hs..h; force = integral of M*Ec*y/EI dy dx
            return np.sum(M_at(z)*Ec/EI*(h0**2 - hsv**2)/2)*dx*k, np.sum(h0 > 0.05)*dx
        taus = []
        for z1, z2 in zip(zs[:-1], zs[1:]):
            t1 = t_end*(z1 - z0)/(135 - z0); t2 = t_end*(z2 - z0)/(135 - z0)
            (n1, w), (n2, _) = Nskin(z1, t1), Nskin(z2, t2)
            taus.append(abs(n2 - n1)/(z2 - z1)/w)
        tau = 1.3*max(taus)
        lab, z, t, m = worst[1]
        print('  scarf %d-135 mm, skin 0 -> %.2f mm a side (steel core at 135 mm: %.2f mm): weakest %s %.1fx at %.0f mm (bar %s); bond peak about %.1f MPa -> %.1fx on 15 MPa' % (
            z0, t_end, max(2*(tmax - t_end), 0), lab, m, z, '3x' if lab == 'steel' else '4x', tau, 15/tau))
    # just above 135 mm, all carbon in the existing loft (22x18% -> 40x20% over 40 mm, solid), bend radius 55 mm
    for s in (0, 5, 10, 15):
        f = s/40; c = 22 + 18*f; tc = (3.96 + 4.04*f)/c; z = 135 + 55*np.sin(s/55)
        h = half_t(c, tc); I = k*np.sum(2*h**3/3)*(c*(x[1]-x[0]))
        print('  carbon loft %2d mm along the bend: %.1fx' % (s, Sc/(M_at(z)*h.max()/I)))
