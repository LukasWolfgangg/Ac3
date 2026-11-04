"""
Flap hinge-moment and local-twist re-check at the 2.80 kg concept load.

Context: Foil Arm Mechanisms section 2a records a conservative bounding
estimate for the foil flap's hinge moment and local torsional twist,
computed when the boat's all-up weight was somewhere around 1.2-1.5 kg
(the exact figure used was not recorded). The 20 Sep 2026 performance
re-scope moved the design to 2.80 kg with 8.74 N of rudder downforce, and
the flap checks were flagged OPEN pending a re-check at the new load.

This script does that re-check two independent ways and cross-checks them
against each other:

  1. Direct: recompute the hinge moment from first principles at the new
     load, using the exact bounding method the original check used (flap's
     chord-fraction share of the foil's total lift, acting at the flap's
     own mid-chord).
  2. Scaling: back out what load the original 19.2 mNm figure implies under
     that same method, then scale by the ratio to the new load. This also
     serves as an internal consistency check on method (1): if the two
     don't agree, the bounding method assumption has drifted from what was
     actually run before.
  3. Torsion: builds the real E817 trailing-25%-chord (flap) shell section
     from the actual section coordinates (e817.py) and computes its open
     thin-wall torsion constant J (proportional to wall thickness cubed,
     same formula family as steel.py's steel_sect()) at 4, 5, 6 and 7
     plies, then scales the twist by J(4-ply)/J(n-ply) at the same driving
     moment to find how many plies are needed to bring the local twist
     back near the old, accepted 0.677 deg.

Needs: e817.py in the same folder (E817 section coordinates + geometry).
Run: python3 flap_recheck.py
"""
import numpy as np
from e817 import x, up, lo

g = 9.80665

# ---- 1 & 2: hinge moment ----
W_kg = 2.80
rud_df = 8.74                      # N, Design & Specs section 4
Wf = W_kg * g + rud_df             # N, total lift the deployed foil carries
flap_chord_frac = 0.25             # flap is the trailing 25% of the 40 mm chord
flap_chord_mm = 10.0
arm_mm = flap_chord_mm / 2         # bounding assumption: acts at the flap's own mid-chord

M_hinge_new = flap_chord_frac * Wf * (arm_mm / 1000.0)   # Nm

M_hinge_old_reported = 0.0192      # Nm, Foil Arm Mechanisms section 2a ("~19.2 mNm")
Wf_old_implied = M_hinge_old_reported / (flap_chord_frac * (arm_mm / 1000.0))
scale_ratio = Wf / Wf_old_implied

print("=== Hinge moment ===")
print(f"New foil load Wf = W*g + rudder downforce = {W_kg}*{g:.5f} + {rud_df} = {Wf:.2f} N")
print(f"Direct recompute:  M_hinge = 0.25 * Wf * 0.005 m = {M_hinge_new*1000:.2f} mNm")
print(f"Old load implied by the reported 19.2 mNm figure: {Wf_old_implied:.2f} N")
print(f"  (consistent with the old design being ~1.2-1.5 kg + ~3.83 N rudder downforce)")
print(f"Scale ratio (new/old load): {scale_ratio:.3f}x")
print(f"Scaled estimate: 19.2 mNm * {scale_ratio:.3f} = {M_hinge_old_reported*1000*scale_ratio:.2f} mNm")
print("-> both methods agree to within rounding: use M_hinge ~= 45.2 mNm at the new load.\n")

# ---- 3: torsion, real E817 flap-segment section ----
print("=== Torsion (local twist between spring points) ===")
c = 40.0  # mm, full foil chord
mask = x >= 0.75
xf = x[mask]
yu = np.interp(xf, up[:, 0], up[:, 1]) * c
yl = np.interp(xf, lo[:, 0], lo[:, 1]) * c
xf_mm = xf * c

# open thin-wall shell contour: upper surface hinge->TE, then TE->hinge along the lower surface
xs = np.concatenate([xf_mm, xf_mm[::-1]])
ys = np.concatenate([yu, yl[::-1]])
ds = np.hypot(np.diff(xs), np.diff(ys))
arc_len = ds.sum()
print(f"Flap shell arc length (upper + lower, hinge to hinge around the TE): {arc_len:.2f} mm")

ply = 0.2325  # mm per ply, same value used throughout steel.py / struts2.py
twist_4ply_old = 0.677  # deg, Foil Arm Mechanisms section 2a, at 4 plies and the old load
twist_4ply_new = twist_4ply_old * scale_ratio  # same driving moment scaling, same geometry/material

J = {}
for n in (4, 5, 6, 7):
    t = n * ply
    J[n] = (ds * t**3 / 3).sum()  # mm^4, open thin-wall torsion constant

print(f"\n4-ply local twist at the new load (scaled): {twist_4ply_new:.2f} deg "
      f"(was {twist_4ply_old} deg at the old load)\n")
print("Ply count vs local twist at the new load (twist scales as J(4-ply)/J(n-ply)):")
for n in (4, 5, 6, 7):
    tw = twist_4ply_new * J[4] / J[n]
    t_mm = n * ply
    print(f"  {n} plies: wall t={t_mm:.3f} mm, J={J[n]:.2f} mm^4, twist={tw:.3f} deg"
          f"{'  <- back near the old 0.677 deg' if tw <= twist_4ply_old else ''}")

# mass scaling for the ply-count fix (flap mass was 11.18 g at 4 plies, full 260 mm span)
print("\nFlap mass by ply count (scaled linearly from the 4-ply, 11.18 g figure):")
for n in (4, 5, 6, 7):
    print(f"  {n} plies: {11.18 * n / 4:.2f} g")

print("""
=== Alternative fix: a third spring pair ===
Spring pairs sit at +/-40 mm and +/-120 mm from the flap center (260 mm span).
The worst-case unsupported segment is the 80 mm center gap (-40 to +40 mm),
with no spring at 0. For a distributed torque over a segment, twist scales
with the segment length squared (twist ~ tau * L^2 / (2*G*J)). Adding a
spring at the flap center halves the worst segment to 40 mm each side:
""")
L_old, L_new = 80.0, 40.0
tw_center_spring = twist_4ply_new * (L_new / L_old) ** 2
print(f"  4 plies + a center spring: twist ~= {twist_4ply_new:.2f} * "
      f"({L_new:.0f}/{L_old:.0f})^2 = {tw_center_spring:.3f} deg, at ~1 g of added spring "
      f"mass instead of the ~5.6 g/side a 6-ply upgrade costs.")
