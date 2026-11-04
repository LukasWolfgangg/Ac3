import numpy as np, io, contextlib
kt=0.514444; rho=999.; rho_a=1.2; g=9.81; nu=1e-6
CAL=18.4/25.7 # calibrate strut model to the project's published 25-kt figure
def cf(Re): return 0.075/(np.log10(max(Re,1e4))-2)**2
def form(tc): return 1+2*tc+60*tc**4
def strut(V,c,L,tc): return CAL*2*cf(V*c/nu)*form(tc)*0.5*rho*V*V*c*L
Re_pts=np.array([100e3,134e3,206e3,247e3,298e3,400e3]); cd_pts=np.array([0.0146,0.0122,0.0093,0.0082,0.0074,0.0065]) # 27 Sep 2026: main foil section E226 (NeuralFoil, Ncrit 9, at the operating CL for each Re); was E817: Re [103e3,288e3,412e3,514e3], CD [0.0257,0.0142,0.0105,0.0088]
def cd0f(Re): return np.interp(np.log(Re),np.log(Re_pts),cd_pts)
LWL=0.98 # 28 Oct 2026: Hull_v2 rev 4 waterline length at 2.80 kg (was 0.9, Hull_v1). Only the pre-takeoff hull-drag Froude number uses it.
PIVOT_Y=0.1146 # 28 Oct 2026: arm pivot at 84% of the Hull_v2 half-beam at the pivot station (136.45 mm, measured on the rev 4 loft, 531 mm from the transom) -> 114.6 mm off the centreline; 86% does not fit the rev D gimbal. Was 0.86*0.118 (101.5 mm, scaled from the AC75 loft). Cant torque still uses reach (pivot-to-foil) only.
# 28 Oct 2026 (evening): the rudder T-foil load for PERFORMANCE now comes from the pitch balance while foiling (cg_layout.py,
# Design & Specs section 8), speed by speed: weight about the main foil, the sail drive / drag couple (drive at the centre of
# effort CE above the waterline, drag at Z_DRAG) and the E226 section pitching moment. Positive = downforce. The righting
# moment, heel cap, foil load, rudder drag and takeoff speed follow it. The 175/550 rule (8.74 N at 2.80 kg) stays as
# rud_df, the STRUCTURAL basis (design cant torque 11.77 Nm, steel.py hybrid(), the blade check); Cfg(pitch=False) restores
# the old performance model.
RUD_STRUCT = 0.5101   # 29 Oct 2026: STRUCTURAL rudder downforce per unit weight = the pitch-balance load on the top reach of the design
# window (12 kt of wind, TWA 105 deg, 22.15 kt; top_reach.py): 14.01 N / 27.47 N at 2.80 kg. Replaces the 175/550 rule (0.318, 8.74 N).
X_CG, X_FOIL, X_RUD, Z_DRAG = 0.4818, 0.5264, 0.020, -0.200   # m from the transom / waterline: all-up CG (Weight Budget row 18,
# 30 Oct 2026, after rudder rev B; mast at 600 mm assumed; was 0.4907 on the 28 Oct layout, cg_layout.py), main foil quarter chord (CAD),
# rudder stock (CAD, Hull_v2 Rudder_A), drag centre (assumed)
CM_E226 = {133000: [(-0.090, -0.0127), (0.152, -0.0388), (0.251, -0.0473), (0.323, -0.0503), (0.417, -0.0457), (0.495, -0.0373), (0.580, -0.0297), (0.670, -0.0239)],
           206000: [(-0.020, -0.0254), (0.123, -0.0299), (0.198, -0.0333), (0.272, -0.0367), (0.399, -0.0391), (0.495, -0.0353), (0.581, -0.0294), (0.669, -0.0240)],
           298000: [(-0.005, -0.0278), (0.096, -0.0231), (0.157, -0.0234), (0.226, -0.0259), (0.368, -0.0320), (0.487, -0.0331), (0.582, -0.0298), (0.669, -0.0247)]}   # NeuralFoil, Ncrit 9 (CL, CM about c/4)
def cm_foil(cl, Re):
    def at(tab):
        for (c0, m0), (c1, m1) in zip(tab, tab[1:]):
            if c0 <= cl <= c1: return m0 + (m1 - m0) * (cl - c0) / (c1 - c0)
        return tab[0][1] if cl < tab[0][0] else tab[-1][1]
    res = sorted(CM_E226); Re = min(max(Re, res[0]), res[-1])
    for r0, r1 in zip(res, res[1:]):
        if r0 <= Re <= r1: return at(CM_E226[r0]) + (at(CM_E226[r1]) - at(CM_E226[r0])) * (Re - r0) / (r1 - r0)

def r_hull(V):
    Fr=V/np.sqrt(g*LWL)
    return np.interp(Fr,[0,0.3,0.4,0.5,0.6,0.8,1.2],[0.02,0.05,0.10,0.14,0.15,0.12,0.08])

class Cfg:
    def __init__(s,name,Wkg=2.0,reach=0.300,A=0.30,tc=0.15,ac=0.045,bc=0.038,
                 Lw=0.247,Lb=0.119,Sf=0.012,fc=0.040,fb=0.300,rigAR=2.4,CD0s=0.06,es=0.80,
                 ballast=0.0,yp=None,note="",pitch=True,xcg=X_CG):
        s.__dict__.update(locals()); del s.s
        s.W=Wkg*g
        s.rud_df=s.W*RUD_STRUCT # STRUCTURAL basis only (top reach since 29 Oct 2026; was W*175/550); performance uses s.rud(V)
        s.dRM=0.0; s._cap_fixed=None; s._R=None; s._rc={}
        s.yp=PIVOT_Y if yp is None else yp
        s.RM=(s.W+s.rud_df)*(reach+s.yp)+ballast # 27 Sep 2026: lever = pivot offset from the centreline + pivot-to-foil reach. Earlier 27 Sep correction: the leeward foil's lift must also cancel the rudder's downforce reaction, and that reaction acts at the same cant-offset lever as the weight term (moment balance about the CG); was s.W*reach+ballast (weight only)
        s.CE=0.25*np.sqrt(A/0.176)*np.sqrt(rigAR/2.4)
    # heel cap: setting c.cap (e.g. 1e9 for "uncapped") fixes it; otherwise it follows the righting moment at each speed
    @property
    def cap(s): return s._cap_fixed if s._cap_fixed is not None else s.RM/s.CE
    @cap.setter
    def cap(s,v): s._cap_fixed=v
    def rud(s,V):
        """rudder T-foil downforce (N) for pitch balance at boat speed V (m/s); the old 175/550 rule if pitch=False"""
        if not s.pitch: return s.W*175/550   # the old performance model
        key=(round(float(V),4),s.CE,getattr(s,'mult',1.0),s.xcg,s.W)
        if key in s._rc: return s._rc[key]
        q=0.5*rho*V*V; R=0.0
        for _ in range(12):
            s._R=R
            try: D=s.drag(V)
            finally: s._R=None
            Mf=-cm_foil((s.W+R)/(q*s.Sf),V*s.fc/nu)*q*s.Sf*s.fc        # Nm, bow down
            Rn=(s.W*(s.xcg-X_FOIL)+D*(s.CE-Z_DRAG)+Mf)/(X_FOIL-X_RUD)
            if abs(Rn-R)<1e-5: R=Rn; break
            R=Rn
        s._rc[key]=R
        return R
    def RMv(s,V): return (s.W+s.rud(V))*(s.reach+s.yp)+s.ballast+s.dRM
    def capv(s,V): return s._cap_fixed if s._cap_fixed is not None else s.RMv(V)/s.CE
    def drag(s,V,foilborne=True):
        q=0.5*rho*V*V
        R=s._R if s._R is not None else s.rud(V)
        Wf=s.W+R # main foil carries weight + rudder downforce
        AR=s.fb**2/s.Sf; CL=min(Wf/(q*s.Sf),0.9)
        foil=(cd0f(V*s.fc/nu)+CL**2/(np.pi*0.85*AR))*q*s.Sf
        Sr=0.0036; CLr=min(abs(R)/(q*Sr),0.6)
        rud=(np.interp(V/kt,[5,14,20,25],[0.0132,0.0076,0.0064,0.0057])+CLr**2/(np.pi*0.85*4))*q*Sr
        d=foil+strut(V,s.ac,s.Lw,s.tc)+strut(V,s.bc,s.Lb,s.tc)+rud
        if not foilborne:
            Lf=q*s.Sf*CL
            d+=r_hull(V)*max(Wf-Lf,0)
        return d
    def sail(s,Va,CL):
        q=0.5*rho_a*Va*Va*s.A
        return q*CL, q*(s.CD0s+CL**2/(np.pi*s.rigAR*s.es))
    def drive(s,V,tws,twa,cap=None):
        cap=s.capv(V) if cap is None else cap
        ux=tws*kt*np.cos(np.radians(twa))+V; uy=tws*kt*np.sin(np.radians(twa))
        Va=np.hypot(ux,uy); b=np.arctan2(uy,ux)
        CLs=np.linspace(0,1.3,131); L,D=s.sail(Va,CLs)
        dr=L*np.sin(b)-D*np.cos(b); sd=L*np.cos(b)+D*np.sin(b)
        ok=sd<=cap
        return (dr[ok].max() if ok.any() and dr[ok].max()>0 else 0.0)
    def speed(s,tws,twa):
        best=0.0
        for V in np.arange(3.0*kt,18,0.05):
            if s.drive(V,tws,twa)>=s.drag(V): best=V/kt
        return best
    def best_upwind(s,tws): return max(s.speed(tws,a) for a in (50,55,60,65))
    def takeoff_V(s,CL=0.55):
        V=np.sqrt(2*s.W/(rho*s.Sf*CL))
        for _ in range(20): V=np.sqrt(2*(s.W+s.rud(V))/(rho*s.Sf*CL))
        return V
    def foils_from(s,margin=1.10):
        Vto=s.takeoff_V()
        Vs=np.linspace(0.3,Vto,40)
        for tws in np.arange(4,26,0.25):
            if all(max(s.drive(V,tws,a) for a in (80,90,100))>=margin*s.drag(V,foilborne=False) for V in Vs):
                return tws
        return None
    def twist12(s):
        q=0.5*rho*(12*kt)**2; M=0.11*q*s.Sf*s.fc
        base=0.036*(0.036*0.388)**2; idx=s.ac*(s.ac*s.tc)**2/base
        return M*(0.90/3.30)/idx
