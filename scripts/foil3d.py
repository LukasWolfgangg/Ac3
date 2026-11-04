"""27 Sep 2026: 3D check of the main T-foil (300 mm span, 40 mm chord, elliptical tip caps, projected 0.011627 m2).
foil.tools analyze_wing, AeroBuildup (NeuralFoil medium) + VLM loading, water rho 999 mu 1e-3, constant lift 36.21 N,
6.5-14.5 kt. No free-surface / ventilation effect (both sections alike). Numbers below are copied from the two runs."""
import io, contextlib, numpy as np
with contextlib.redirect_stdout(io.StringIO()):
    import r250c as R
import opt
KT=opt.kt; S3=0.011627
V=np.array([3.3439,3.8583,4.3728,4.8872,5.4017,5.9161,6.4305,6.9450,7.4594])
RUN={'E817':dict(a=[4.467,2.986,1.656,0.610,-0.208,-0.872,-1.425,-1.893,-2.292],
                 CD=[0.038648,0.031483,0.026742,0.023514,0.021130,0.019246,0.017702,0.016407,0.015288],
                 CDp=[0.021991,0.022090,0.021020,0.019843,0.018653,0.017522,0.016469,0.015491,0.014585],
                 CMle=[-0.2243,-0.1941,-0.1763,-0.1642,-0.1556,-0.1496,-0.1456,-0.1430,-0.1413],CLmax=0.984,e=0.768),
     'E226':dict(a=[4.987,2.861,1.772,1.250,0.935,0.709,0.525,0.362,0.218],
                 CD=[0.029322,0.020701,0.016166,0.013436,0.011604,0.010364,0.009426,0.008790,0.008306],
                 CDp=[0.012680,0.011300,0.010448,0.009751,0.009141,0.008627,0.008202,0.007871,0.007605],
                 CMle=[-0.1637,-0.1402,-0.1180,-0.0982,-0.0827,-0.0709,-0.0620,-0.0553,-0.0502],CLmax=0.933,e=0.767)}
CL=np.array([0.5576,0.4188,0.3261,0.2610,0.2137,0.1781,0.1508,0.1293,0.1121])
TAB={'E817':(np.array([103e3,288e3,412e3,514e3]),np.array([0.0257,0.0142,0.0105,0.0088])),   # 28 Sep 2026: explicit E817 table (opt.py now carries E226, so copying opt's table ran E226 twice)
     'E226':(np.array([100e3,134e3,206e3,247e3,298e3,400e3]),np.array([0.0146,0.0122,0.0093,0.0082,0.0074,0.0065]))}
def vpp_foil(c,v):
    q=0.5*opt.rho*v*v; R=c._R if c._R is not None else c.rud(v); Wf=c.W+R; AR=c.fb**2/c.Sf; cl=min(Wf/(q*c.Sf),0.9)   # 28 Oct 2026: pitch-balance rudder load (opt.Cfg.rud)
    return (opt.cd0f(v*c.fc/opt.nu)+cl**2/(np.pi*0.85*AR))*q*c.Sf
def make(sec,use3d):
    Re,cd=TAB[sec]; opt.cd0f=lambda r,Re=Re,cd=cd: np.interp(np.log(r),np.log(Re),cd)
    c=R.mk3()
    if use3d:
        D3=np.array(RUN[sec]['CD'])*0.5*opt.rho*V**2*S3
        c._R=c.W*175/550; ratio=D3/np.array([vpp_foil(c,v) for v in V]); c._R=None   # the 3D runs were at the basis lift 36.21 N: ratio at the same lift
        base=c.drag
        def drag(V_,foilborne=True,c=c,base=base,ratio=ratio):
            f=vpp_foil(c,V_); return base(V_,foilborne)+(np.interp(V_,V,ratio)-1)*f
        c.drag=drag; c.ratio=ratio
    return c
if __name__=='__main__':
    q=lambda k:0.5*opt.rho*(k*KT)**2
    for sec in ('E817','E226'):
        r=RUN[sec]; D=np.array(r['CD'])*0.5*opt.rho*V**2*S3
        print(sec,'3D foil drag N at 6.5..14.5 kt:',D.round(2),' alpha:',r['a'][0],'->',r['a'][-1],'spread %.2f deg, mid %.2f'%(r['a'][0]-r['a'][-1],(r['a'][0]+r['a'][-1])/2),
              ' CM c/4 @14.5kt %.3f'%(r['CMle'][-1]+0.25*CL[-1]*0.04/0.03928),' CLmax_wing %.2f (takeoff CL %.3f, x%.2f)'%(r['CLmax'],CL[0],r['CLmax']/CL[0]))
        for k in (6.5,10,12,14.5):
            v=k*KT; print('   %4.1f kt: 3D %.2f N'%(k,np.interp(v,V,D)))
        for use3d in (False,True):
            c=make(sec,use3d)
            lab='3D-corrected' if use3d else '2D (VPP)    '
            extra=(' ratio 3D/VPP foil drag '+str(np.round(c.ratio,2))) if use3d else ''
            print('  ',lab,'foils from %.2f kt | upwind 10/11/12 %s | reach 10/12 %.2f %.2f | total drag @10kt %.2f N'%(
                c.foils_from(),[round(float(c.best_upwind(t)),2) for t in (10,11,12)],c.speed(10,90),c.speed(12,90),c.drag(10*KT)),extra,flush=True)
