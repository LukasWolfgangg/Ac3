import io, contextlib, json, sys
with contextlib.redirect_stdout(io.StringIO()):
    from steel3 import *
import steel2, steel
REACH=0.250
def best_hyb(Wkg,reach=REACH,sj=280,cands=((38,0.17),(40,0.18),(42,0.19),(44,0.20))):
    for cc,tcc in cands:
        best=None
        for cs in np.arange(20,40,1.0):
            for tcs in np.arange(0.08,0.21,0.01):
                r=hybrid(cs,tcs,sj,cc,tcc,6,Wkg,reach,MAT['17-4PH H1025'])
                if r['tw']>1.0 or r['marg_s']<3 or r['marg_c']<4: continue
                if best is None or r['drag10']<best['drag10']: best=r
        if best: best['cc']=cc; best['tcc']=tcc; return best
    return None
def mk2(hyb,Wkg,A,reach=REACH):
    c=CfgH('x',Wkg=Wkg,reach=reach,A=A,tc=0.17,ac=0.038,bc=0.032,rigAR=2.4,CD0s=0.06,es=0.80); c.hyb=hyb; return c
if __name__=="__main__":
    out=[]
    for Wkg in (2.4,2.8,3.2,3.6):
        h=best_hyb(Wkg)
        if h is None: print(Wkg,"no feasible hybrid"); continue
        print("W",Wkg,"hybrid",{k:round(float(h[k]),2) for k in ('cs','tcs','cc','tcc','tw','marg_s','marg_c','mass','m_s','Mj')},flush=True)
        for A in (0.30,0.36,0.42,0.48):
            c=mk2(h,Wkg,A); ff=c.foils_from()
            row=dict(W=Wkg,A=A,ff=ff,cap=c.cap,CE=c.CE,RM=c.RM,to=c.takeoff_V()/kt,d10=c.drag(10*kt),u10=c.best_upwind(10),u11=c.best_upwind(11),u12=c.best_upwind(12),r12=c.speed(12,90))
            out.append(row); print({k:(round(float(v),2) if v is not None else None) for k,v in row.items()},flush=True)
    json.dump(out,open('r250.json','w'))
