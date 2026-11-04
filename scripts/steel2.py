from steel import *
import steel as S
fRe=lambda Re: float(np.interp(np.log(Re),np.log([113e3,195e3,290e3]),[1.22,1.035,0.977]))
def strut_c(V,c,L,tc): return strut(V,c,L,tc)*fRe(V*c/1e-6)
S.strut=strut_c
W,reach=2.4,0.300
def refd(V): return strut_c(V,0.038,0.247,0.17)
print("carbon 38 mm 17pct arm drag (NF-corrected): 6kt %.2f  8kt %.2f  10kt %.2f  12kt %.2f N"%tuple(refd(v*kt) for v in (6,8,10,12)))
def sweep(cc,tcc,nc,mat,sjs,tw_lim=1.0,marg=3.0):
    out=[]
    for sj in sjs:
        best=None
        for cs in np.arange(14,40,1.0):
            for tcs in np.arange(0.06,0.21,0.01):
                r=hybrid(cs,tcs,sj,cc,tcc,nc,W,reach,MAT[mat])
                if r['tw']>tw_lim or r['marg_s']<marg or r['marg_c']<4.0: continue
                if best is None or r['drag10']<best['drag10']: best=r
        out.append((sj,best))
    return out
ref10=refd(10*kt)
for mat in ('17-4PH H1025',):
  for (cc,tcc,nc) in ((38,0.17,6),(44,0.20,6)):
    print("\n== steel:",mat," dry carbon %d mm t/c %.2f %d plies"%(cc,tcc,nc))
    for sj,b in sweep(cc,tcc,nc,mat,(149,200,247,280)):
        if b is None: print(sj,"infeasible"); continue
        d={v:strut_c(v*kt,b['cs']/1000,b['Ls']/1000,b['tcs'])+(strut_c(v*kt,cc/1000,b['Lc']/1000,tcc) if b['Lc']>0 else 0) for v in (6,8,10,12)}
        print(" joint %3d: steel %2.0f mm x %.2f (t %.1f) wet %3.0f, carbon wet %3.0f | twist %.2f | steel margin %.1fx | drag 6/8/10/12kt: %.2f/%.2f/%.2f/%.2f (ref %.2f/%.2f/%.2f/%.2f) | saving@10 %.0f%% | mass %.0f g (steel %.0f)"%(
           sj,b['cs'],b['tcs'],b['ts'],b['Ls'],b['Lc'],b['tw'],b['marg_s'],d[6],d[8],d[10],d[12],refd(6*kt),refd(8*kt),refd(10*kt),refd(12*kt),100*(1-d[10]/ref10),b['mass'],b['m_s']))
