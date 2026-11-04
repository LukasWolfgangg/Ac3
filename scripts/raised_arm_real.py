"""Raised-windward-arm righting-moment contribution, using the real-yacht swing direction (27 Sep 2026).

AC75 Class Rule V3.01 Figure 12.2 / Rule 12.7 / Rule 32.3:
  - cant axis longitudinal, 2050 mm off centreline, 520 mm above MWP (inside the topsides)
  - cant angle 10.0 deg = arm hanging down, wing tucked under the hull; wing level at ~41.4 deg
  - racing range 10-90 deg while down; fully raised >115 deg, capability 119.5 deg
  - cant angle increases as the arm swings OUTBOARD and UP (the only direction that
    does not drive the arm through the hull)
ETNZ FCS test: 70 deg foiling -> 120 deg fully raised, a 50 deg arc in about 3 s.

Here: rigid rotation of the traced arm (arm_pts.csv at 250 mm reach) about its pivot,
outboard-and-up (+theta, x outboard, z up). dRM = sum m*g*dx (CG shift to windward),
relative to the model's baseline of both arms down and mirror-symmetric (CG on centreline).
"""
import numpy as np
from steel import path_from_foil

g = 9.81
p, ds, tx, tz, s = path_from_foil(250/225)   # foil (index 0) ... pivot (index -1), mm
pivot = p[-1]
rel = p - pivot                               # positions relative to pivot
S = s[-1]; SJ = 135.0                         # steel/carbon joint at 135 mm path length (28 Sep 2026: Foilarm_v3; was 280 mm)

# lumped masses at the foil (kg)
m_foil = 0.222 + 0.0045 + 7*0.002             # steel fixed element (E226) + carbon flap (solid laminate) + 7 hinge springs (Weight Budget, 28 Sep 2026; was 0.240 + 0.01118 E817)
# distributed along the path
m_steel = 0.0506; m_carb = 0.0469              # CAD-measured, Foilarm_v3 (was 0.136 / 0.0248 with the 280 mm joint)
mid = (rel[:-1] + rel[1:]) / 2
w_steel = np.clip((SJ - s[:-1]) / ds, 0, 1) * ds
w_carb = ds - w_steel
pts = [(m_foil, rel[0][None, :], np.array([1.0]))]
pts.append((m_steel, mid, w_steel / w_steel.sum()))
pts.append((m_carb, mid, w_carb / w_carb.sum()))
M = sum(m for m, _, _ in pts)

def rot(xz, th):
    c, s_ = np.cos(np.radians(th)), np.sin(np.radians(th))
    return np.stack([xz[:, 0]*c - xz[:, 1]*s_, xz[:, 0]*s_ + xz[:, 1]*c], axis=1)

def dRM(th):
    out = 0.0
    for m, xz, w in pts:
        out += m * g * ((rot(xz, th)[:, 0] - xz[:, 0]) * w).sum() / 1000
    return out

def foil_pos(th):
    return rot(rel[:1], th)[0]

print(f"pivot rel. foil (mm): {pivot.round(1)}, foil rel. pivot: {rel[0].round(1)}, "
      f"foil-vector angle from vertical {np.degrees(np.arctan2(rel[0][0], -rel[0][1])):.1f} deg")
print(f"rotating mass per side {M*1000:.0f} g (foil lump {m_foil*1000:.0f} g, steel arm {m_steel*1000:.0f} g, carbon arm {m_carb*1000:.1f} g)")
print(f"\n{'sweep':>6} {'dRM Nm':>7} {'% of 13.20':>10} {'foil x from pivot':>18} {'foil z vs pivot':>16}")
for th in (0, 30, 45, 50, 60, 70, 78, 90, 100, 110, 120, 140):
    fx, fz = foil_pos(th)
    print(f"{th:6d} {dRM(th):7.3f} {100*dRM(th)/13.20:9.1f}% {fx:17.0f}  {fz:15.0f}")
ths = np.arange(0, 181, 0.5); vals = [dRM(t) for t in ths]
i = int(np.argmax(vals)); print(f"\nmax dRM {vals[i]:.3f} Nm at sweep {ths[i]:.1f} deg; dRM returns to 0 at sweep "
      f"{ths[np.where((np.array(vals) < 0) & (ths > 10))[0][0]]:.1f} deg")

# the withdrawn 'up+inboard' case (rotation -theta): it drives the foil deeper and under the hull
for th in (30, 60, 90):
    fx, fz = rot(rel[:1], -th)[0]
    print(f"old -theta case, {th} deg: foil at x={fx:.0f} mm, z={fz:.0f} mm rel. pivot (down position z={rel[0][1]:.0f})")

# AC75 comparison (Figure 12.2 read-off, +/- ~25 mm): wing junction ~4350 mm from the axis,
# ~2 deg outboard of vertical at cant 10 deg.
r, a10 = 4350.0, 2.0
for c in (10, 41.4, 50, 70, 90, 119.5):
    a = np.radians(a10 + c - 10)
    print(f"AC75 cant {c:5.1f}: wing {2050 + r*np.sin(a):5.0f} mm from CL, {520 - r*np.cos(a):6.0f} mm vs MWP")
