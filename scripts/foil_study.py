"""27 Sep 2026: is the main T-foil (E817, 40 x 300 mm) still right after the pivot/sail changes?
Section drag from NeuralFoil (foil.tools), Ncrit 9, at the chosen case's operating CL for each Re.
E817 is opt.py's existing table; E226 (10.2%, 0.8% camber) is the best-ranked alternative."""
import io, contextlib, numpy as np
with contextlib.redirect_stdout(io.StringIO()):
    import r250c as R
    from r250 import best_hyb
import opt, steel, struts2
from concurrent.futures import ProcessPoolExecutor
E817=(opt.Re_pts.copy(), opt.cd_pts.copy())
E226=(np.array([100e3,134e3,206e3,247e3,298e3,400e3]), np.array([0.0146,0.0122,0.0093,0.0082,0.0074,0.0065]))
# conservative: turbulent water (Ncrit 5) cuts E817's low-CL drag too; E226/E817 ratio there is 0.0087/0.0130 = 0.67
E226c=(opt.Re_pts.copy(), opt.cd_pts*0.67)
TABLES={'E817':E817,'E226':E226,'E226 (Ncrit5 ratio)':E226c}
def setsec(name):
    Re,cd=TABLES[name]; opt.cd0f=lambda r,Re=Re,cd=cd: np.interp(np.log(r),np.log(Re),cd)
def run(args):
    sec,chord,span,hyb=args
    setsec(sec)
    c=R.C2('x',Wkg=2.8,reach=0.25,A=0.44,tc=0.17,ac=0.038,bc=0.034,rigAR=2.4,CD0s=0.06,es=0.80,
           Sf=chord*span*1e-6,fc=chord/1000,fb=span/1000); c.hyb=hyb
    q=0.5*opt.rho*(10*opt.kt)**2
    return (sec,chord,span,round(c.foils_from(),2),round(c.takeoff_V()/opt.kt,2),[round(float(c.best_upwind(t)),2) for t in (10,11,12)],
            round(float(c.drag(10*opt.kt)),2), round(float(c.speed(12,90)),2))
if __name__=='__main__':
    jobs=[('E817',40,300,R.h),('E226',40,300,R.h),('E226 (Ncrit5 ratio)',40,300,R.h)]
    jobs+=[('E226',ch,sp,R.h) for ch,sp in ((34,300),(46,300),(40,260),(40,340),(36,340),(34,350))]
    with ProcessPoolExecutor(2) as ex:
        for r in ex.map(run,jobs): print("%-20s chord %d span %d: foils from %s kt (Vto %s) upwind 10/11/12 %s | drag@10kt %s N | reach@12 %s"%r,flush=True)
    # arm: E226's pitching moment is ~1/3 of E817's, and arm twist was what sized the steel section
    base=struts2.M14
    for k in (1.0,0.33):
        steel.M14=base*k
        h=best_hyb(2.8)
        print("M14 x%.2f: best hybrid"%k,{x:round(float(h[x]),3) for x in ('cs','tcs','cc','tcc','tw','marg_s','marg_c','mass','drag10')} if h else None,flush=True)
