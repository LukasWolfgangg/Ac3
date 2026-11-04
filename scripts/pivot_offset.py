"""27 Sep 2026: arm pivot moved to 86% of the hull half-beam (AC75 cant axis is at 2050/2450 = 84%).
Half-beam at the pivot is not measured yet: Hull_v1 is lofted from the AC75 at ~1:20.7, and the AC75 rule puts the
hull perimeter 2.40-2.50 m from the centreline -> 116-121 mm here, ~118 mm nominal -> pivot ~101 mm off the centreline.
Case A: arm unchanged (250 mm pivot-to-foil) -> foil moves out to ~351 mm from the centreline.
Case B: foil kept at 250 mm from the centreline -> arm reach (pivot-to-foil) shrinks to ~149 mm."""
import io, contextlib
with contextlib.redirect_stdout(io.StringIO()):
    import r250c as R
    from r250 import best_hyb, mk2
from steel import hybrid, MAT
g=9.81
HB=0.118; P=0.86*HB
base=R.mk3(); L=base.W+base.rud_df
base.RM=L*0.25; base.cap=base.RM/base.CE # baseline = pivot on the centreline (opt.py now defaults to the 101.5 mm offset)
print(f"load {L:.2f} N, pivot offset {P*1000:.1f} mm (half-beam 116-121 mm -> {0.86*116:.0f}-{0.86*121:.0f} mm)")
def show(tag,c):
    print(f"{tag}: RM {c.RM:.2f} Nm cap {c.cap:.1f} N | upwind 8.75/10/11/12: "
          f"{[round(float(c.best_upwind(t)),2) for t in (8.75,10,11,12)]} | reach 10/12: {c.speed(10,90):.2f} {c.speed(12,90):.2f} | foils from {c.foils_from():.2f}")
show("baseline (pivot on CL)",base)
# Case A
cA=R.mk3(); cA.RM=L*(0.25+P); cA.cap=cA.RM/cA.CE; show("A: arm unchanged, foil at %.0f mm"%((0.25+P)*1000),cA)
cU=R.mk3(); cU.cap=1e9; print("  unlimited RM upwind 10/11/12:",[round(float(cU.best_upwind(t)),2) for t in (10,11,12)])
print(f"  A: cant torque about pivot unchanged {L*0.25:.2f} static / {1.3*L*0.25:.2f} design; extra hull transverse moment {L*P:.2f} Nm")
# Case A weight scan: lighter boat now has the righting moment it lacked (r250.py scan settings)
for W in (2.2,2.4,2.6,2.8):
    h=best_hyb(W)
    for A in (0.30,0.36,0.40,0.42,0.48):
        c=mk2(h,W,A); c.RM=(c.W+c.rud_df)*(0.25+P); c.cap=c.RM/c.CE
        ff=c.foils_from()
        print(f"  A scan W {W} A {A}: RM {c.RM:.2f} foils from {ff} upwind 10/11/12 {c.best_upwind(10):.2f} {c.best_upwind(11):.2f} {c.best_upwind(12):.2f} steel marg {h['marg_s']:.2f}",flush=True)
# Case B
rB=0.25-P
hB=best_hyb(2.8,reach=rB)
print(f"B: arm reach {rB*1000:.0f} mm; cant torque {L*rB:.2f} static / {1.3*L*rB:.2f} design (was 9.05/11.77)")
if hB: print("  B best hybrid:",{k:round(float(hB[k]),3) for k in ('cs','tcs','cc','tcc','tw','marg_s','marg_c','mass','Mj','drag10')})
hA=R.h; print("  chosen hybrid now:",{k:round(float(hA[k]),3) for k in ('cs','tcs','tw','marg_s','marg_c','mass','Mj','drag10')})
cB=mk2(hB,2.8,0.40,reach=rB); cB.RM=L*0.25; cB.cap=cB.RM/cB.CE; show("B: foil at 250 mm, shorter arm",cB)
