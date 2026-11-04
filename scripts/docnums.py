import io, contextlib, numpy as np
with contextlib.redirect_stdout(io.StringIO()):
    import e817
    from r250c import *
from steel import path_from_foil, hybrid, MAT, steel_sect
from struts2 import M14   # 28 Sep 2026: twist now scaled from the E226 pitching moment M14 (0.29 Nm), joint at 135 mm (r250c.JOINT)
x=e817.x; th=e817.th
m=x<=0.75
A75=np.trapezoid(th[m],x[m]); A=e817.A
print("E817 area/c2: full %.4f, forward75 %.4f"%(A,A75))
c=40.0
# span: 300 mm with elliptical tip caps -> approx effective span; use rectangle 300 and 290
for span in (300,290,280):
    print("span",span,"fixed element steel 17-4PH (7.78): %.1f g ; carbon steel 7.85: %.1f g"%(A75*c*c*span*7.78e-3, A75*c*c*span*7.85e-3))
# arm path at reach 250
for r in (225,250,300):
    p,ds,tx,tz,s=path_from_foil(r/225.0); print("reach",r,"path length %.0f mm"%s[-1])
# twist at speeds
tw14=h['tw']; print("hybrid twist at M14 = %.2f Nm: %.3f deg (steel %.3f carbon %.3f)"%(M14,tw14,h['tw_s'],h['tw_c']))
S=0.012; cc=0.04
cases=[(6.5,-0.033),(10,-0.035),(12,-0.028),(14.5,-0.023)]   # E226 CM (NeuralFoil) at the operating points; was E817 (-0.086/-0.097/-0.105/-0.112)
for vk,CM in cases:
    V=vk*kt; q=0.5*999*V*V; M=abs(CM)*q*S*cc
    print("%.1f kt: q %.0f Pa  M %.3f Nm  twist %.2f deg"%(vk,q,M,tw14*M/M14))
# heel cap binding TWS (upwind)
for t in (7,8,8.75,9,9.5,10):
    cx=mk3(); u=cx.best_upwind(t); c2=mk3(); c2.cap=1e9; uu=c2.best_upwind(t)
    print("TWS %.2f capped %.2f uncapped %.2f"%(t,u,uu))
# steel tang mass
tang=steel_sect(20,0.18); print("tang area mm2 %.1f  -> 40 mm long %.1f g"%(tang['A'],tang['A']*40*7.78e-3))
print("steel lower arm to joint (constant section, %.0f mm): %.1f g"%(JOINT,h['As']*JOINT*7.78e-3))
