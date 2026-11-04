import io, contextlib
with contextlib.redirect_stdout(io.StringIO()):
    from steel2 import *
import opt
def best_h(cc,tcc,nc,sj,cmin=20):
    best=None
    for cs in np.arange(cmin,40,1.0):
        for tcs in np.arange(0.06,0.21,0.01):
            r=hybrid(cs,tcs,sj,cc,tcc,nc,W,reach,MAT['17-4PH H1025'])
            if r['tw']>1.0 or r['marg_s']<3 or r['marg_c']<4: continue
            if best is None or r['drag10']<best['drag10']: best=r
    return best
class CfgH(opt.Cfg):
    hyb=None
    def drag(s,V,foilborne=True):
        base=opt.Cfg.drag(s,V,foilborne)-opt.strut(V,s.ac,s.Lw,s.tc)      # remove plain arm
        if s.hyb is None: arm=strut_c(V,s.ac,s.Lw,s.tc)
        else:
            b=s.hyb; arm=strut_c(V,b['cs']/1000,b['Ls']/1000,b['tcs'])+(strut_c(V,s.ac,b['Lc']/1000,s.tc) if b['Lc']>0 else 0)
        return base+arm
def mk(hyb,Wkg=2.4,A=0.36):
    c=CfgH('x',Wkg=Wkg,reach=0.300,A=A,tc=0.17,ac=0.038,bc=0.032,rigAR=2.4,CD0s=0.06,es=0.80); c.hyb=hyb; return c
print("Steel min chord 20 mm (Re>=~100k at 10 kt)")
cases={'carbon 38/17 (recommended)':None}
for cc,tcc in ((38,0.17),(44,0.20)):
    for sj in (200,280):
        b=best_h(cc,tcc,6,sj); 
        if b: cases['steel %2.0fx%.2f to %d mm + dry carbon %d/%.2f'%(b['cs'],b['tcs'],sj,cc,tcc)]=b; 
for W,A in ((2.4,0.36),(3.2,0.42)):
    print("\nAUW %.1f kg, sail %.2f m2"%(W,A))
    for k,b in cases.items():
        c=mk(b,W,A); ff=c.foils_from()
        print(" %-52s arm@10kt %.2f N | boat drag@10kt %.2f N | foils from %.2f kt | upwind 10/11/12 TWS: %.2f/%.2f/%.2f kt | reach 12kt %.2f"%(k,strut_c(10*kt,c.ac,0.247,c.tc) if b is None else strut_c(10*kt,b['cs']/1000,b['Ls']/1000,b['tcs']),c.drag(10*kt),ff,c.best_upwind(10),c.best_upwind(11),c.best_upwind(12),c.speed(12,90)))
print()
for k,b in cases.items():
    if b: print(k, "twist %.2f (steel %.2f) steel margin %.1fx joint moment %.1f Nm mass %.0f g (steel %.0f) J %.0f mm4 Ith %.0f mm4"%(b['tw'],b['tw_s'],b['marg_s'],b['Mj'],b['mass'],b['m_s'],b['Js'],b['Ith']))
