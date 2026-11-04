"""29 Oct 2026: glue-line shear along a steel-to-carbon overlap on the straight strut, by a shear-lag model, for
(A) the builder's simplification: straight steel of constant full section ending flat at 135 mm, carbon sleeve on the outside
    thickening from a feather edge at 95 mm to t135 per side (the strut gets thicker there), and
(B) the scarf: steel thinning to an edge at 135 mm under a carbon skin that keeps the outside shape.
Equivalent-rectangle section (22 mm wide; 2.95 mm thick reproduces the measured steel Iy 47.1 mm4). One face skin carries axial
force N(z); the steel carries M - 2*N*d; glue: G 1000 MPa over 0.2 mm (epoxy plus the glass isolation ply), ASSUMED.
N'' = (G*b/ta) * (eps_skin - eps_steel_surface);  N(95) = 0;  at 135 mm the steel carries nothing (all the moment in the carbon).
Moment from the top-reach side force (38.0 N x1.3, uniform on the 135 mm wet length). Glue judged against 15 MPa wet (assumed)."""
import numpy as np
F, KL, LW = 38.0, 1.3, 135.0
M = lambda z: KL*F*z**2/(2*LW)          # N mm
b, Ga, ta, Es = 22.0, 1000.0, 0.2, 197e3
T_EQ = 2.95                              # equivalent full steel thickness (mm)
def solve(ts_f, tc_f, Ec, z0=95.0, z1=135.0, n=2001):
    z = np.linspace(z0, z1, n); h = z[1]-z[0]
    ts = np.maximum(ts_f(z), 0.03); tc = np.maximum(tc_f(z), 0.005); d = ts/2 + tc/2
    K = Ga*b/ta
    a = K*(1/(Ec*b*tc) + 12*d/(Es*b*ts**2)); r = -K*6*M(z)/(Es*b*ts**2)
    A = np.zeros((n, n)); rhs = np.zeros(n)
    A[0, 0] = 1; rhs[0] = 0.0
    A[-1, -1] = 1; rhs[-1] = M(z1)/(2*d[-1])
    for i in range(1, n-1):
        A[i, i-1] = 1/h**2; A[i, i] = -2/h**2 - a[i]; A[i, i+1] = 1/h**2; rhs[i] = r[i]
    N = np.linalg.solve(A, rhs)
    tau = np.gradient(N, z)/b
    i = np.argmax(np.abs(tau))
    return abs(tau[i]), z[i], np.mean(np.abs(tau))
for Ec, lab in ((21.3e3, 'woven carbon'), (100e3, 'UD carbon')):
    print('==', lab)
    for t135 in (0.5, 1.0, 1.25, 1.5):
        tp, zp, ta_ = solve(lambda z: T_EQ + 0*z, lambda z, t=t135: t*(z-95)/40, Ec)
        print('  (A) straight steel, carbon sleeve 0 -> %.2f mm a side: glue peak %.1f MPa at %.0f mm (mean %.1f) -> %.1fx on 15 MPa' % (t135, tp, zp, ta_, 15/tp))
    tp, zp, ta_ = solve(lambda z: T_EQ*(135-z)/40, lambda z: (T_EQ/2)*(z-95)/40, Ec)
    print('  (B) scarf, steel thinning to an edge at 135 mm: glue peak %.1f MPa at %.0f mm (mean %.1f) -> %.1fx on 15 MPa' % (tp, zp, ta_, 15/tp))
