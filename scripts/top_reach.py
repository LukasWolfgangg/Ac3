"""29 Oct 2026: top-reach load case for the structural basis. Scans true wind angle at 10-12 kt of wind (the design
window) and prints, at the fastest point of sail, the boat speed, the pitch-balance rudder downforce, the foil load,
the righting moment, the heel cap and the sail's steady side (heeling) force at the operating point.
30 Oct 2026: re-run at the rudder rev B CG (opt.X_CG 0.4818) with the 136 mm wetted rudder blade (r250c.LB): the top reach is now
21.86 kt at TWA 105 with 13.64 N on the rudder, 41.11 N on the foil and 13.36 Nm of design cant torque; at 22.15 kt the rudder
load is 14.04 N. The structural basis stays at the 29 Oct 2026 values (22.15 kt, 14.01 N, 41.48 N, 13.48 Nm), which envelope it.
5 Nov 2026: re-run at the CG of the 240 mm blade (opt.X_CG 0.4819) with the 156 mm wetted, 34 -> 20 mm blade (r250c.LB, r250c.BC): the top
reach is 21.95 kt at TWA 105 with 13.57 N on the rudder, 41.04 N on the foil and 13.34 Nm of design cant torque; at 22.15 kt the rudder
load is 13.83 N. Still inside the structural basis."""
import io, contextlib, numpy as np
with contextlib.redirect_stdout(io.StringIO()):
    import r250c as R
from opt import kt
def side_force(c, V, tws, twa):
    """sail side force (N) at the lowest sail CL that drives the boat at V (the trimmed operating point)"""
    ux=tws*kt*np.cos(np.radians(twa))+V; uy=tws*kt*np.sin(np.radians(twa))
    Va=np.hypot(ux,uy); b=np.arctan2(uy,ux)
    CLs=np.linspace(0,1.3,1301); L,D=c.sail(Va,CLs)
    dr=L*np.sin(b)-D*np.cos(b); sd=L*np.cos(b)+D*np.sin(b)
    ok=(sd<=c.capv(V))&(dr>=c.drag(V))
    if not ok.any(): return None, None, Va/kt, np.degrees(b)
    i=np.argmax(ok); return sd[i], CLs[i], Va/kt, np.degrees(b)
for mult in (1.0, 1.3):
    c=R.mk3(mult=mult)
    print('drag x%.1f' % mult)
    for tws in (10, 11, 12, 13, 14):
        best=(0,None)
        for twa in range(60, 171, 5):
            v=c.speed(tws, twa)
            if v>best[0]: best=(v, twa)
        v, twa = best
        # refine to 1 deg
        for a in range(twa-4, twa+5):
            vv=c.speed(tws, a)
            if vv>v: v, twa = vv, a
        V=v*kt
        sd, CLs, AWS, AWA = side_force(c, V, tws, twa)
        print('  TWS %4.1f: best %.2f kt at TWA %d (AWS %.1f kt, AWA %.0f); rudder %.2f N, foil %.2f N, RM %.2f Nm, cap %.1f N, sail side force %s N at sail CL %s; static cant %.2f Nm, design x1.3 %.2f Nm' % (
            tws, v, twa, AWS, AWA, c.rud(V), c.W+c.rud(V), c.RMv(V), c.capv(V), '%.1f'%sd if sd else '-', '%.2f'%CLs if CLs is not None else '-', (c.W+c.rud(V))*0.25, 1.3*(c.W+c.rud(V))*0.25), flush=True)
# drag breakdown and foil pitching moment at the 12 kt top reach (inputs to gimbal_revE_calcs.py, rev E3)
from opt import rho, nu, cm_foil
c=R.mk3(); vk=22.15; V=vk*kt; q=0.5*rho*V*V
b=R.breakdown(c,vk); CL=(c.W+c.rud(V))/(q*c.Sf); cm=cm_foil(CL,V*c.fc/nu)
print('22.15 kt breakdown (N):', {k:round(float(v),3) for k,v in b.items()}, ' foil CL %.4f, E226 CM %.4f, pitching moment %.3f Nm nose-down; structural rudder rule opt.RUD_STRUCT x W = %.2f N' % (CL, cm, -cm*q*c.Sf*c.fc, c.rud_df))
