import io, contextlib, json
with contextlib.redirect_stdout(io.StringIO()):
    from r250 import *
from opt import cd0f, nu
W=2.8; reach=0.25; A=0.44 # 27 Sep 2026: sail 0.44 m2 (was 0.40), adopted after the pivot move
# chosen case: steel wet arm 22 mm x 18%, joint 135 mm (top of the straight vertical run, Foilarm_v3, decided 27 Sep 2026; was 280 mm), carbon upper arm 40 mm x 20% t/c, 6 plies
# 28 Sep 2026: joint and wetted length (steel.Wetted) updated so this script reproduces Design & Specs section 11 for the 135 mm joint
from steel import Wetted
JOINT=135.0
h=hybrid(22,0.18,JOINT,40,0.20,6,W,reach,MAT['17-4PH H1025'])
class C2(CfgH):
    mult=1.0
    def drag(s,V,foilborne=True): return s.mult*CfgH.drag(s,V,foilborne)
LB=0.1358 # 30 Oct 2026: rudder blade wetted length at the design ride = 220 mm below the hull - 84.2 mm of air under the stern
# (rudder_limit.py, Hull_v2); was 0.119, inferred from the old 150 mm blade's drag calibration
def mk3(A=A,W=W,mult=1.0,CEscale=1.0):
    c=C2('x',Wkg=W,reach=reach,A=A,tc=0.17,ac=0.038,bc=0.034,Lw=Wetted/1000,Lb=LB,rigAR=2.4,CD0s=0.06,es=0.80); c.hyb=h; c.mult=mult
    c.CE*=CEscale; return c   # 28 Oct 2026: the heel cap follows c.CE and the speed-dependent righting moment (opt.Cfg.capv)
c=mk3()
print("W %.2f kg  A %.2f  righting moment %.2f Nm  structural design cant torque %.2f Nm  CE %.3f m  heel cap %.1f N (design-margined %.1f N)"%(W,A,c.RM,1.3*(c.W+c.rud_df)*reach,c.CE,c.cap,1.3*c.cap))
print("takeoff V (CL 0.55): %.2f kt ; foils from TWS: %.2f kt"%(c.takeoff_V()/kt,c.foils_from()))
print("rudder downforce: structural basis %.2f N (top reach, opt.RUD_STRUCT, 29 Oct 2026); performance, from the pitch balance (opt.Cfg.rud, 28 Oct 2026):"%c.rud_df)
for vk in (6.5,8,10,12,14.5,16):
    V=vk*kt; print("  %.1f kt: rudder downforce %.2f N, foil load %.2f N, righting moment %.2f Nm, heel cap %.1f N"%(vk,c.rud(V),c.W+c.rud(V),c.RMv(V),c.capv(V)))
def breakdown(c,vk):
    V=vk*kt; q=0.5*rho*V*V; R=c.rud(V); Wf=c.W+R; AR=c.fb**2/c.Sf; CL=min(Wf/(q*c.Sf),0.9)
    foil=(cd0f(V*c.fc/nu)+CL**2/(np.pi*0.85*AR))*q*c.Sf
    b=c.hyb
    arm=strut_c(V,b['cs']/1000,b['Ls']/1000,b['tcs'])
    blade=strut_c(V,c.bc,c.Lb,c.tc)
    Sr=0.0036; CLr=min(abs(R)/(q*Sr),0.6)
    rud=(np.interp(vk,[5,14,20,25],[0.0132,0.0076,0.0064,0.0057])+CLr**2/(np.pi*0.85*4))*q*Sr
    return dict(V=vk,rud=R,CLfoil=CL,foil=foil,arm=arm,blade=blade,rudT=rud,tot=foil+arm+blade+rud, hull=c.drag(V,False)-c.drag(V,True))
for vk in (6.5,8,10,12,14.5):
    b=breakdown(c,vk); print({k:round(float(v),3) for k,v in b.items()})
# speeds
print("upwind best (TWA 50-65) at TWS 8.75/10/11/12: ",[round(c.best_upwind(t),2) for t in (8.75,10,11,12)])
print("reach TWA90 at 10/12 : ",round(c.speed(10,90),2),round(c.speed(12,90),2)," downwind TWA135 at 10/12:",round(c.speed(10,135),2),round(c.speed(12,135),2))
# which TWA best at 10,11,12
for t in (10,11,12):
    print(t,[ (a,round(c.speed(t,a),2)) for a in (50,55,60,65,75,90)])
# heel cap binding: compare capped vs uncapped best upwind
for t in (10,12,14,16,18,20):
    u_cap=c.best_upwind(t)
    c2=mk3(); c2.cap=1e9; u_unc=c2.best_upwind(t)
    print("TWS %d: upwind capped %.2f uncapped %.2f"%(t,u_cap,u_unc))
# sensitivity
for m in (0.7,1.0,1.3):
    cc=mk3(mult=m); print("drag x%.1f: upwind 10/11/12: %.2f %.2f %.2f ; foils from %.2f"%(m,cc.best_upwind(10),cc.best_upwind(11),cc.best_upwind(12),cc.foils_from()))
for s in (0.8,1.0,1.2):
    cc=mk3(CEscale=s); print("CE x%.1f: upwind 10/11/12: %.2f %.2f %.2f"%(s,cc.best_upwind(10),cc.best_upwind(11),cc.best_upwind(12)))
# weight sensitivity at the chosen sail area
for w in (2.6,2.8,3.0):
    hh=hybrid(22,0.18,JOINT,40,0.20,6,w,reach,MAT['17-4PH H1025'])
    cc=C2('x',Wkg=w,reach=reach,A=A,tc=0.17,ac=0.038,bc=0.034,Lw=Wetted/1000,Lb=LB,rigAR=2.4,CD0s=0.06,es=0.80); cc.hyb=hh
    print("W %.1f: ff %.2f  upwind10/11/12 %.2f %.2f %.2f  steel margin %.1f  twist %.2f  Vto %.2f"%(w,cc.foils_from(),cc.best_upwind(10),cc.best_upwind(11),cc.best_upwind(12),hh['marg_s'],hh['tw'],cc.takeoff_V()/kt))
print("hybrid:",{k:round(float(v),2) for k,v in h.items()})
# carbon-only baseline at same W/A for comparison (same wetted length)
c0=C2('x',Wkg=W,reach=reach,A=A,tc=0.17,ac=0.038,bc=0.034,Lw=Wetted/1000,Lb=LB,rigAR=2.4,CD0s=0.06,es=0.80)
print("carbon-only 38/17 arm: ff %.2f upwind %.2f %.2f %.2f"%(c0.foils_from(),c0.best_upwind(10),c0.best_upwind(11),c0.best_upwind(12)))
