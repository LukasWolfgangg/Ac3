"""Rudder rev B linkage check (29 Oct 2026, PROPOSED numbers; geometry from Hull_v2 version 21, Rudder_A).

Rev B moves both rudder servos below deck (Mechanisms - Rudder section 2d):
  - rake: AGF RC A50BHLW, shaft athwartships at x 60, z 5, 8 mm horn pointing up; a 29.5 mm rod runs aft
    along the centreline to a ball on the carrier top at x 30.5, z 13.
  - steering: Savox SW-0250MG to port, shaft fore and aft at y -60, z 12, 7 mm horn hanging down; a 60 mm rod
    runs athwartships to a ball on a 15 mm tiller low on the stock (x 35, z 5).
  - the whole rudder rakes about a trunnion at x 27, z -14 (7 mm forward of the stock at x 20), so one pin runs
    straight through the carrier.
Hull frame: x from the transom, y to starboard, z from the design waterline, mm.

The script works out the servo angles for the rake and steering ranges, how much rake moves the steering with the
steering servo held, and the rake load at the servo. The rake moment (1.55 Nm with x1.3 at 22 kt and 5 deg of rake,
1.19 Nm steady) is the Design & Specs / Rudder Mechanism value about the trunnion, 226 mm above the T-foil.
The CAD check in Fusion (servo horns driven, the other horn locked) agrees: rake -5.49 / +5.49 deg at horn
-19.0 / +18.7 deg, steering -0.12 deg at full rake, rudder +-20 deg at horn -46.6 / +47.8 deg.
"""
import math
import numpy as np
from scipy.optimize import brentq

d2r = math.radians
PIVOT = np.array([27.0, 0.0, -14.0])        # trunnion axis (along y)
STOCK = np.array([20.0, 0.0, -14.0])        # steering axis at zero rake, at the trunnion height


def Ry(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# ---------------- steering
R_TILLER, Z_BALL, L_ROD, R_HORN = 15.0, 5.0, 60.0, 7.0
SERVO_S = np.array([STOCK[0] + R_TILLER, -L_ROD, Z_BALL + R_HORN])      # horn shaft, axis along x


def tiller_ball(delta, rake):
    local = (STOCK - PIVOT) + Rz(delta) @ np.array([R_TILLER, 0, 0]) + np.array([0, 0, Z_BALL - PIVOT[2]])
    return PIVOT + Ry(rake) @ local


def horn_ball(phi):
    return SERVO_S + np.array([0, R_HORN * math.sin(phi), -R_HORN * math.cos(phi)])


ROD = np.linalg.norm(tiller_ball(0, 0) - horn_ball(0))
horn_for = lambda de, th: brentq(lambda p: np.linalg.norm(tiller_ball(de, th) - horn_ball(p)) - ROD, -1.5, 1.5)
rudder_for = lambda ph, th: brentq(lambda d: np.linalg.norm(tiller_ball(d, th) - horn_ball(ph)) - ROD, -0.8, 0.8)

print('steering: rod %.1f mm, tiller %.0f mm, horn %.0f mm' % (ROD, R_TILLER, R_HORN))
for de in (-20, -10, 10, 20):
    print('  rudder %+3d deg needs horn %+6.2f deg' % (de, math.degrees(horn_for(d2r(de), 0))))
print('  rake moves the steering (servo held at neutral):')
for th in (-5.5, -2.7, 2.7, 5.5):
    print('    rake %+4.1f deg -> rudder %+.3f deg' % (th, math.degrees(rudder_for(0, d2r(th)))))
for de in (-20, 20):
    ph = horn_for(d2r(de), 0)
    print('    at rudder %+d deg: rake +5.5 -> %+.3f, rake -5.5 -> %+.3f deg' % (
        de, math.degrees(rudder_for(ph, d2r(5.5))) - de, math.degrees(rudder_for(ph, d2r(-5.5))) - de))

# steering load and resolution
HINGE_MAX = 0.060                                    # Nm, hull-borne near stall (Rudder Mechanism section 3)
ratio = math.degrees(horn_for(d2r(20), 0)) / 20.0
t_servo = HINGE_MAX / (R_TILLER / 1000) * (R_HORN / 1000)
print('  ratio about %.2f:1; worst hinge moment %.0f mNm is %.2f kg cm at the servo (SW-0250MG 5.0 kg cm: %.0fx)' % (
    ratio, HINGE_MAX * 1000, t_servo / 0.0980665, 5.0 / (t_servo / 0.0980665)))
for db in (3, 5):
    print('  dead band %d us = about %.2f deg at the rudder (0.067 deg per us at the servo)' % (db, db * 0.067 / ratio))

# ---------------- rake
LUG = np.array([30.5, 0.0, 13.0])
RAKE_S, R_RAKE_HORN = np.array([60.0, 0.0, 5.0]), 8.0
lug = lambda th: PIVOT + Ry(th) @ (LUG - PIVOT)
rake_ball = lambda ps: RAKE_S + np.array([R_RAKE_HORN * math.sin(ps), 0, R_RAKE_HORN * math.cos(ps)])
RROD = np.linalg.norm(lug(0) - rake_ball(0))
horn_rake = lambda th: brentq(lambda p: np.linalg.norm(lug(th) - rake_ball(p)) - RROD, -1.2, 1.2)
print('\nrake: rod %.1f mm, horn %.0f mm' % (RROD, R_RAKE_HORN))
for th in (-5.5, -2.5, 2.5, 5.5):
    print('  rake %+4.1f deg needs horn %+6.2f deg' % (th, math.degrees(horn_rake(d2r(th)))))
lever = LUG[2] - PIVOT[2]
for M, lab in ((1.55 / 1.3, 'steady, 22 kt, 5 deg of rake'), (1.55, 'design, x1.3')):
    F = M / (lever / 1000)
    T = F * R_RAKE_HORN / 1000 / 0.0980665
    print('  %s: %.2f Nm -> %.1f N in the rod, %.2f kg cm at the servo; A50BHLW (18 kg cm at 7.4 V) %.0f%%, SW-0250MG (5.0) %.0f%%' % (
        lab, M, F, T, 100 * T / 18.0, 100 * T / 5.0))
F = 1.55 / 1.3 / (lever / 1000)
print('  the SW-0250MG would need a horn of %.1f mm or less to hold under a third of its stall torque' % (5.0 * 0.0980665 / 3 / F * 1000))
