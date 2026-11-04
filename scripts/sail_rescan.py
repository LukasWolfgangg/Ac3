# 27 Sep 2026: sail area re-scan after the pivot move (RM lever 351 mm).
# 28 Oct 2026: re-run with the pitch-balance rudder load (opt.Cfg.rud); the heel cap printed is at 15 kt of boat speed.
# 30 Oct 2026: re-run with the all-up CG at 481.8 mm (opt.X_CG, rudder rev B) and the 136 mm wetted rudder blade (r250c.LB).
# 28 Sep 2026: re-run for the 135 mm joint. Every weight now uses the chosen arm (22 mm x 18% steel to the 135 mm joint, 40 mm x 20% carbon above):
# with the joint on the vertical run the wet steel section no longer depends on weight, and the carbon upper arm is dry, so re-sizing it for weight
# changes its mass, not the drag. (Was: arm re-sized per weight with r250.best_hyb at the 280 mm joint.)
import io, contextlib
with contextlib.redirect_stdout(io.StringIO()):
    import r250c as R
    from steel import hybrid, MAT, Wetted
from opt import kt
from concurrent.futures import ProcessPoolExecutor
def run(args):
    W,A=args
    h=R.h if W==2.8 else hybrid(22,0.18,R.JOINT,40,0.20,6,W,0.25,MAT['17-4PH H1025'])
    c=R.C2('x',Wkg=W,reach=0.25,A=A,tc=0.17,ac=0.038,bc=0.034,Lw=Wetted/1000,Lb=R.LB,rigAR=2.4,CD0s=0.06,es=0.80); c.hyb=h
    c2=R.C2('x',Wkg=W,reach=0.25,A=A,tc=0.17,ac=0.038,bc=0.034,Lw=Wetted/1000,Lb=R.LB,rigAR=2.4,CD0s=0.06,es=0.80); c2.hyb=h; c2.cap=1e9
    return (W,A,c.foils_from(),[round(float(c.best_upwind(t)),2) for t in (10,11,12)],round(float(c2.best_upwind(12)),2),round(float(c.capv(15*kt)),1),round(c.CE,3),round(float(h['marg_s']),2))
jobs=[(W,A) for W in (2.4,2.6,2.8,3.0) for A in (0.40,0.44,0.48,0.52,0.56,0.60)]
with ProcessPoolExecutor(8) as ex:
    for r in ex.map(run,jobs): print("W %.1f A %.2f: foils from %s upwind 10/11/12 %s (uncapped@12 %s) cap@15kt %s N CE %s m steel %s"%r,flush=True)
