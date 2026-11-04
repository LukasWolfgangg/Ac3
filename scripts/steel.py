import numpy as np, io, contextlib
from struts2 import *          # up (E864 half-thickness table), sect_t, arm_check, path pts, M14, g, kt
with contextlib.redirect_stdout(io.StringIO()):
    import opt
from opt import strut, rho, g, kt, RUD_STRUCT

Wetted=135.0    # mm of arm path below the waterline at the design ride. 28 Sep 2026: the whole 135 mm straight steel run is wetted and the carbon stays dry (joint at 135 mm, Foilarm_v3, decided 27 Sep 2026; the design ride sits ~112 mm higher). Was 247 mm, inferred, with the joint at 280 mm

def steel_sect(c,tc):
    """solid steel foil section: E864 outline thinned to t/c=tc. c in mm."""
    k=tc/0.388
    x=up[::-1,0]*c; t=2*up[::-1,1]*k*c
    A=np.trapezoid(t,x); xc=np.trapezoid(x*t,x)/A
    Ic=np.trapezoid((x-xc)**2*t,x); Ith=np.trapezoid(t**3/12,x); J=np.trapezoid(t**3/3,x)
    return dict(A=A,Ic=Ic,Ith=Ith,J=J,t=c*tc)

def path_from_foil(reach_scale):
    p=np.vstack([pts,[[0.0,0.0]]]).copy(); p[:,0]*=reach_scale
    p=p[::-1]                                   # foil first
    d=np.diff(p,axis=0); ds=np.hypot(d[:,0],d[:,1])
    return p,ds,d[:,0]/ds,d[:,1]/ds,np.concatenate([[0],np.cumsum(ds)])
def split_int(reach_scale,sj):
    p,ds,tx,tz,s=path_from_foil(reach_scale)
    sm=(s[:-1]+s[1:])/2; w=np.clip((sj-s[:-1])/ds,0,1)      # fraction of each step inside steel
    Iz_s=(tz**2*ds*w).sum(); Ix_s=(tx**2*ds*w).sum()
    Iz_t=(tz**2*ds).sum(); Ix_t=(tx**2*ds).sum()
    j=np.searchsorted(s,sj); 
    return Iz_s,Ix_s,Iz_t-Iz_s,Ix_t-Ix_s,p[min(j,len(p)-1)],s[-1]

MAT={'17-4PH H1025':dict(E=197e3,G=77e3,Y=1000.), '316L annealed':dict(E=193e3,G=77e3,Y=205.)}

def hybrid(cs,tcs,sj,cc,tcc,nc,W,reach,mat,Kj=1.25,marg_steel=3.0):
    """steel wet strut (chord cs mm, t/c tcs) up to path length sj (mm from foil); carbon (cc,tcc,nc plies) beyond."""
    rs=reach/0.225
    Mdes=1.3*W*g*(1+RUD_STRUCT)*reach   # structural design moment: the deployed foil carries weight + the top-reach rudder downforce (29 Oct 2026; was (1+175/550), and 1.3*W*g*reach before 22 Sep 2026)
    Iz_s,Ix_s,Iz_c,Ix_c,pj,S=split_int(rs,sj)
    ss=steel_sect(cs,tcs); cs_=sect_t(cc,nc,tcc)
    f=(nc-1)/nc; E=50e9*(1-f)+15.6e9*f; G=4.5e9*(1-f)+20e9*f
    comp_s=(Iz_s*1e-3)/(mat['E']*1e6*ss['Ic']*1e-12)+(Ix_s*1e-3)/(mat['G']*1e6*ss['J']*1e-12)
    comp_c=(Iz_c*1e-3)/(E*cs_['Ic']*1e-12)+(Ix_c*1e-3)/(G*cs_['J']*1e-12)
    tw_s=np.degrees(M14*comp_s); tw_c=np.degrees(M14*comp_c)
    xoff=abs(pj[0])                                    # mm, horizontal offset foil->joint
    Mj=Mdes*Kj*xoff/(reach*1e3)                        # Nm at the joint
    sig_s=Mj*1e3*(ss['t']/2)/ss['Ith']
    marg_s=mat['Y']/sig_s
    # carbon root check (unchanged arm)
    sig_c=Mdes*1e3*(cc*tcc/2)/cs_['Ith']; allow=373*(1-f)+373*.5*f
    # drag at 10 kt
    V=10*kt
    Ls=min(sj,Wetted); Lc=max(0,Wetted-sj)
    d=strut(V,cs/1000,Ls/1000,tcs)+ (strut(V,cc/1000,Lc/1000,tcc) if Lc>0 else 0)
    m_s=ss['A']*sj*7.85e-3           # g
    m_c=cs_['P']*1e-3*((S-sj)*1e-3)*nc*0.339*1e3
    return dict(cs=cs,tcs=tcs,sj=sj,Ls=Ls,Lc=Lc,xoff=xoff,zj=pj[1],tw=tw_s+tw_c,tw_s=tw_s,tw_c=tw_c,Mj=Mj,sig_s=sig_s,marg_s=marg_s,
                marg_c=allow/sig_c,drag10=d,m_s=m_s,m_c=m_c,mass=m_s+m_c,Js=ss['J'],Ith=ss['Ith'],ts=ss['t'],As=ss['A'])
def carbon_ref(c,tc,n,W,reach):
    rs=reach/0.225; r=arm_check(c,tc,n,rs,1.3*W*g*(1+RUD_STRUCT)*reach)   # same load basis as hybrid()
    r['drag10']=strut(10*kt,c/1000,Wetted/1000,tc); return r
if __name__=="__main__":
    W,reach=2.4,0.300
    ref=carbon_ref(38,0.17,6,W,reach); print("carbon ref 38/17%/6:",{k:round(v,3) for k,v in ref.items()})
    p,ds,tx,tz,s=path_from_foil(reach/0.225)
    for sj in (100,149,180,200,247,280,320):
        j=np.searchsorted(s,sj); print("s=%3d from foil: x=%.0f z=%.0f"%(sj,p[j,0],p[j,1]))
