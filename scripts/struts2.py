import numpy as np
exec(open('arm.py').read().split("pts=np.loadtxt")[0])          # loads E864 contour + sect()
cont0=cont.copy()
def sect_t(c,n,tc,ply=0.2325):
    global cont
    k=tc/0.388
    cont=cont0.copy(); cont[:,1]*=k
    r=sect(c,n,ply); cont=cont0
    return r
pts=np.loadtxt('arm_pts.csv', delimiter=',')
def path_integrals(reach_scale):
    p=np.vstack([pts,[[0.0,0.0]]]).copy(); p[:,0]*=reach_scale
    d=np.diff(p,axis=0); ds=np.hypot(d[:,0],d[:,1])
    tx=d[:,0]/ds; tz=d[:,1]/ds
    return (tz**2*ds).sum(),(tx**2*ds).sum(),ds.sum()
kt=0.514444
M14=0.29 # Nm nose-down pitching moment on the foil at 14 kt: E226 (CM about -0.023, S 0.012, c 0.04), 27 Sep 2026; was 1.43           # Nm nose-down pitching moment on the foil at 14 kt (E817, S 0.012, c 0.04); foil unchanged
def arm_check(c,tc,n,reach_scale,Mdesign,Ltot=None):
    Iz,Ix,S=path_integrals(reach_scale)
    s=sect_t(c,n,tc)
    f=(n-1)/n                                     # 1 ply 0/90 + rest bias
    E=50e9*(1-f)+15.6e9*f; G=4.5e9*(1-f)+20e9*f
    comp=(Iz*1e-3)/(E*s['Ic']*1e-12)+(Ix*1e-3)/(G*s['J']*1e-12)
    tw14=np.degrees(M14*comp)
    # in-plane (athwartships) root bending about chord axis: sigma = M*(t/2)/Ith
    thick=c*tc
    sig=Mdesign*1e3*(thick/2)/s['Ith']            # MPa
    allow=373*(1-f)+373*0.5*f                     # bias plies at half the 0/90 allowable
    mass=s['P']*1e-3*(S*1e-3)*n*0.339*1e3
    return dict(tw14=tw14,sig=sig,margin=allow/sig,mass=mass,S=S)
if __name__=="__main__":
    # calibration: current arm 36 mm, 39%, 3 plies, 225 reach (scale 1), design torque 3.52 Nm
    r=arm_check(36,0.388,3,1.0,3.52); print("current arm:",{k:round(v,2) for k,v in r.items()})
