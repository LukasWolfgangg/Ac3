# Break-even drag: chosen case (r250c.mk3) with a multiplier on all foil-borne and pre-takeoff drag (first run 20 Oct 2026).
import io, contextlib
with contextlib.redirect_stdout(io.StringIO()):
    import r250c as R
for m in (1.0,1.3,1.6,2.0,2.5,3.0):
    c=R.mk3(mult=m)
    print("drag x%.1f: foils from %.2f kt  upwind at 10 kt %.2f  upwind at 12 kt %.2f"%(m,c.foils_from(),c.best_upwind(10),c.best_upwind(12)),flush=True)
