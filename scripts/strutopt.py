import numpy as np, io, contextlib
from struts2 import *
with contextlib.redirect_stdout(io.StringIO()):
    import opt
from opt import strut, kt, g, rho
def blade_check(cb,tc,rud_df,Lb=0.15):
    for nb in (1,2,3,4):
        s=sect_t(cb,nb,tc)
        M=rud_df*1.3*Lb                                   # Nm, with the same 1.3 design margin
        sig=M*1e3*(cb*tc/2)/s['Ith']                      # MPa
        rot=np.degrees(M*1e3*Lb*1e3/(2*50e9*1e-6*s['Ith']*1e-0))  # crude cantilever P L^2/(2EI) -> rad; consistent units below
        # units: M[Nm]->N*mm: M*1e3 ; Ith mm^4 ; E N/mm2 = 50e3 ; L mm
        rot=np.degrees((M*1e3)*(Lb*1e3)/(2*50e3*s['Ith']))
        if 373/sig>=4 and rot<=0.6: return nb,373/sig,rot,s
    return None
def search(W,reach,tw_lim=1.0,marg=4.0,nmax=6,verbose=False):
    rs=reach/0.225; Mdes=1.3*W*g*reach; rud=W*g*175/550
    best=None; rows=[]
    for c in np.arange(30,64,2.0):
        for tc in np.arange(0.08,0.40,0.01):
            for n in range(3,nmax+1):
                r=arm_check(c,tc,n,rs,Mdes)
                if r['tw14']>tw_lim or r['margin']<marg: continue
                b=blade_check(0.84*c/1000*1000,tc,rud)
                if b is None: continue
                nb,bm,brot,bs=b
                V=10*kt
                d=strut(V,c/1000,0.247,tc)+strut(V,0.84*c/1000,0.119,tc)
                m=r['mass']*2+ (bs['P']*1e-3*0.15*nb*0.339*1e3)
                rows.append((d,c,tc,n,r['tw14'],r['margin'],r['mass'],nb,bm,brot,m))
    rows.sort()
    return rows
if __name__=="__main__":
    for W in (2.0,2.5,3.0):
        rows=search(W,0.300)
        print(f"\nAUW {W} kg, reach 300 mm (design torque {1.3*W*g*0.3:.1f} Nm, rudder df {W*g*175/550:.1f} N) — lowest-drag feasible struts (twist@14kt<=1.0 deg, bending margin>=4):")
        print("drag@10kt  chord  t/c  plies  twist14  margin  armMass  bladePlies(margin,rot)")
        for r in rows[:6]:
            print(f"  {r[0]:5.2f} N   {r[1]:4.0f}  {r[2]:.2f}   {r[3]}    {r[4]:.2f}°   {r[5]:4.1f}x   {r[6]:5.1f} g    n={r[7]} ({r[8]:.0f}x, {r[9]:.2f}°)")
        print(f"  ... {len(rows)} feasible combos; current E864 arm+blade drag at 10 kt would be {strut(10*kt,0.036,0.247,0.388)+strut(10*kt,0.030,0.119,0.388):.2f} N")
