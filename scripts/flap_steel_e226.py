"""27 Sep 2026: flap and steel fixed element re-check, E817 vs E226 (40 mm chord, hinge at 75%).
Same method as flap_recheck.py / Foil Arm Mechanisms 2a: open thin-wall torsion, J = sum(t^3 ds)/3, twist for a
40 mm bay at the 45.25 mNm bound hinge moment calibrated from the doc (1.60 deg at J 5.44 mm4 over 80 mm -> 0.40 deg over 40 mm)."""
import numpy as np
from cst import surf
c=40.0; PLY=0.2325; NPLY=4; w=NPLY*PLY; SPAN_FLAP=260.; SPAN=300.
x=np.linspace(0,1,4001)
K40=0.40*5.44  # deg*mm4 for a 40 mm bay at the design hinge moment
for n in ('e817','e226'):
    yu,yl=surf(n,x); t=(yu-yl)*c; X=x*c
    fw=x>=0.75; fx=X[fw]; ft=t[fw]; fyu=yu[fw]*c; fyl=yl[fw]*c
    # open shell of 4 plies where the section is thick enough, solid laminate where it is not
    ds_u=np.hypot(np.diff(fx),np.diff(fyu)); ds_l=np.hypot(np.diff(fx),np.diff(fyl)); tm=(ft[1:]+ft[:-1])/2
    shell=tm>=2*w
    J_shell=(w**3/3)*(ds_u+ds_l)[shell].sum()
    J_solid=(tm[~shell]**3/3*np.diff(fx)[~shell]).sum()
    J=J_shell+J_solid
    arc=(ds_u+ds_l).sum()
    hollow_len=np.diff(fx)[shell].sum()
    A_flap=np.trapezoid(ft,fx)  # mm2 of section aft of the hinge
    # flap volume: shell walls where hollow, solid where not
    vol=((ds_u+ds_l)[shell]*w).sum()+(tm[~shell]*np.diff(fx)[~shell]).sum()
    # steel fixed element 0-75%
    st=~fw; A_st=np.trapezoid(t[st],X[st]); m_st=A_st*SPAN*7.78e-3
    # bending of the steel element as a beam (spanwise), about the chord line through its centroid
    yc=(yu[st]+yl[st])/2*c; tt=t[st]
    zbar=np.trapezoid(yc*tt,X[st])/A_st
    I=np.trapezoid(tt**3/12+tt*(yc-zbar)**2,X[st])
    zmax=max((yu[st]*c-zbar).max(),(zbar-yl[st]*c).max())
    M=1.3*36.21/2*(4*0.15/(3*np.pi))   # Nm, elliptic half-load at its centroid, 1.3 design margin
    sig=M*1e3*zmax/I
    Jst=np.trapezoid(tt**3,X[st])/3
    print(f"{n}: t at hinge {np.interp(0.75,x,t):.2f} mm; flap: hollow over {hollow_len:.1f} of 10 mm, arc {arc:.2f} mm, J {J:.2f} mm4 "
          f"-> twist {K40/J:.2f} deg per 40 mm bay; flap volume {vol:.1f} mm2 x {SPAN_FLAP:.0f} mm = {vol*SPAN_FLAP*1.55e-3:.1f} g at 1.55 g/cm3; "
          f"section area aft of hinge {A_flap:.1f} mm2")
    print(f"      steel element: area {A_st:.1f} mm2 -> {m_st:.0f} g raw (before tip caps); I {I:.1f} mm4, root bending {M:.2f} Nm -> {sig:.0f} MPa "
          f"(yield ~1000, margin {1000/sig:.1f}x); torsion J {Jst:.1f} mm4")
