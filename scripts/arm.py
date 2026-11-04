import numpy as np
up=np.array([[1,0.00778],[0.99842,0.00835],[0.99369,0.01003],[0.98583,0.01271],[0.97491,0.01628],[0.96098,0.02065],[0.94414,0.02581],[0.92449,0.03181],[0.90216,0.03871],[0.87728,0.04656],[0.85002,0.05530],[0.82055,0.06482],[0.78905,0.07492],[0.75572,0.08540],[0.72077,0.09615],[0.68443,0.10712],[0.64693,0.11835],[0.60849,0.12986],[0.56937,0.14161],[0.52981,0.15336],[0.49006,0.16467],[0.45037,0.17495],[0.41100,0.18353],[0.37219,0.18980],[0.33419,0.19335],[0.29724,0.19397],[0.26157,0.19169],[0.22740,0.18670],[0.19496,0.17926],[0.16445,0.16960],[0.13606,0.15793],[0.10997,0.14442],[0.08635,0.12929],[0.06534,0.11281],[0.04708,0.09541],[0.03168,0.07754],[0.01925,0.05967],[0.00985,0.04213],[0.00355,0.02505],[0.00040,0.00831],[0,0]])
# closed contour: upper TE->LE then lower LE->TE
lo=up[::-1][1:].copy(); lo[:,1]*=-1
cont=np.vstack([up,lo,up[:1]])
def sect(c,n,ply=0.2325):
    p=cont*c  # mm
    x,y=p[:,0],p[:,1]
    dx,dy=np.diff(x),np.diff(y); ds=np.hypot(dx,dy); xm=(x[:-1]+x[1:])/2
    P=ds.sum(); xc=(xm*ds).sum()/P
    t=n*ply
    Ic=t*((xm-xc)**2*ds).sum()          # mm^4 chordwise bending (thin wall)
    Ith=t*((( (y[:-1]+y[1:])/2)**2)*ds).sum()  # through-thickness bending
    A=0.5*abs(np.sum(x[:-1]*y[1:]-x[1:]*y[:-1]))
    J=4*A*A*t/P
    return dict(P=P,A=A,t=t,Ic=Ic,Ith=Ith,J=J)
pts=np.loadtxt('arm_pts.csv', delimiter=',')
# order pivot->foil? orient so path runs from foil to pivot
print(pts[:3],pts[-3:],pts.shape)

p=np.vstack([pts,[[0.0,0.0]]])
d=np.diff(p,axis=0); ds=np.hypot(d[:,0],d[:,1]); S=ds.sum()
tx=d[:,0]/ds; tz=d[:,1]/ds
Iz=(tz**2*ds).sum(); Ix=(tx**2*ds).sum()
print("path len",S,"int tz^2",Iz,"int tx^2",Ix)
# rho: check calibration old model
c,n=27,6
s=sect(c,n)
print("27mm n6:",s)
Eold=1.2538e-3*n*c**3   # Nm^2 = E*Ic
print("implied E (GPa) with computed Ic:", Eold/(s['Ic']*1e-12)/1e9)

print("\n--- pitching moment on foil: CM*q*S*c (S=0.012, c=0.04, rho=999)")
Sf,cf=0.012,0.04
Ms={}
for kt,CM in [(5,0.051),(14,0.115),(20,0.13),(25,0.134)]:
    V=kt*0.5144; q=0.5*999*V*V; M=CM*q*Sf*cf; Ms[kt]=M
    print(kt,"kt V=%.2f q=%.0f Pa M=%.2f Nm"%(V,q,M))
mats={'0/90 (E50,G4.5)':(50e9,4.5e9),'50/50 (E33,G12)':(32.8e9,12.25e9),'+-45 (E16,G20)':(15.6e9,20e9)}
def theta(c,n,E,G,M):
    s=sect(c,n)
    comp=(Iz*1e-3)/(E*s['Ic']*1e-12)+(Ix*1e-3)/(G*s['J']*1e-12)
    bend=(Iz*1e-3)/(E*s['Ic']*1e-12); tors=(Ix*1e-3)/(G*s['J']*1e-12)
    return np.degrees(M*comp), np.degrees(M*bend), np.degrees(M*tors)
def mass(c,n,L=0.405): return sect(c,n)['P']*1e-3*L*n*0.339*1000*0  # placeholder
def mass(c,n,L=0.4048): return sect(c,n)['P']/n/0.2325*0+ sect(c,n)['P']*1e-3*L*n*0.339*1e3
for c in (36,27):
    print("\nchord",c)
    for n in (2,3,4,5,6):
        row="n=%d m=%.1f g | "%(n,mass(c,n))
        for k,(E,G) in mats.items():
            th,b,t=theta(c,n,E,G,Ms[20])
            row+="%s: %.2f deg (b%.2f t%.2f) | "%(k.split()[0],th,b,t)
        print(row)
print("\nTheta per Nm (deg/Nm), c=36:")
for n in (2,3,4,5,6):
    print(n,[round(theta(36,n,E,G,1.0)[0],3) for (E,G) in mats.values()])
print("\nTheta at each speed, c=36, n=3,4,6, 0/90 and 50/50 and 45")
for n in (3,4,6):
  for k,(E,G) in mats.items():
    print(n,k,[ (kt,round(theta(36,n,E,G,Ms[kt])[0],2)) for kt in (14,20,25)])
# stresses
s=sect(36,3); print("sect36 n3",s)
for n in (3,):
    s=sect(36,n)
    for kt in (25,):
        M=Ms[kt]; sig=M*1e3*18/s['Ic']  # N/mm2
        tau=M*0.6*1e3/(2*s['A']*s['t'])
        print("n",n,"sigma bend MPa %.1f tau MPa %.1f"%(sig,tau))
