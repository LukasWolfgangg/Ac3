# 27 Sep 2026: section 11 chosen case with the raised-windward-arm righting-moment gain added (see raised_arm_real.py)
import io, contextlib
with contextlib.redirect_stdout(io.StringIO()):
    import r250c as R
for b in (0.0, 0.14, 0.21, 0.31):   # 28 Sep 2026: gains at 78, 70 and 45 deg from raised_arm_real.py with the 135 mm joint (was 0.13, 0.22, 0.35)
    c=R.mk3(); c.dRM=b   # 28 Oct 2026: added to the speed-dependent righting moment (opt.Cfg.RMv)
    print("raised-arm dRM %+.2f Nm: RM@15kt %.2f cap %.1f N  upwind 8.75/10/11/12: %s  reach 10/12: %.2f %.2f"%(
        b,c.RMv(15*R.kt),c.capv(15*R.kt),[round(float(c.best_upwind(t)),2) for t in (8.75,10,11,12)],c.speed(10,90),c.speed(12,90)))
