"""Gimbal rev E3: strength and cant-drive performance at the top-reach structural basis (first-principles, PROPOSED numbers).

29 Oct 2026: the structural basis is raised from the 36.21 N 175/550 rule to the VPP's top reach in the design window
(12 kt of wind, TWA 105 deg, 22.15 kt of boat speed; top_reach.py on opt.py with the pitch-balance rudder load):
  foil load 41.48 N (weight 27.47 N + rudder downforce 14.01 N), static cant torque 41.48 N x 0.25 m = 10.37 Nm,
  design cant torque x1.3 = 13.48 Nm; steady sail side force 38.0 N (the heel cap binds), carried by the leeward strut
  (the sail's centre of effort is estimated within about 10 mm of the strut station, so the rudder takes almost none).
Upset survival case (definition DECIDED 28 Oct 2026, side force updated 29 Oct 2026): the design cant torque plus the steady
  strut side force reversed, 13.48 + 38.0 x 0.2025 = 21.18 Nm; 1.5x on yield for metal, polymer and ply, 2x on failure for carbon.
Design case bars: 3x on yield for metal and polymer, 4x on failure for carbon (Design & Specs section 8).
Rev E3 fixes: two 3 mm sector pins instead of one 4 mm pin; rake-fork cheeks 5 mm instead of 4 mm at the bushings; a 7th ply
  (0/90) root doubler on the carbon arm; the spur pair retuned from 28T/56T (2:1) to 24T/60T (2.5:1) at the same 21 mm centres,
  so the 20D gearbox stays inside its continuous rating at the new design torque. The cross-pin checks now use the standard linear bearing-pressure formula
  (pressure grows with radius in the shaft); the rev E2 script used a uniform-pressure model, which was optimistic.
29 Oct 2026 (evening): rake servo changed to the AGF RC A66BHLW (30 kg.cm at 7.4 V, 66 g, IP67; the Hitec DB778WP was
  picked first and dropped on cost) on the same 5:1 linkage, with a 10.25 mm link and a 4.8 mm 7075 horn hub on the 25T
  spline (Hull_v2); the rake moment includes the
  straight-steel joint's +0.03 Nm; the link pins are checked in single shear (they were wrongly called double shear); the link is 1 mm
  301 stainless, half-hard, because plain 250 MPa steel is 2.96x in pin bearing against the 3x bar."""
import math

KGCM = 0.0980665           # Nm per kg.cm
FOIL, RUD, F_SIDE = 41.48, 14.01, 38.0           # N at the top reach (VPP, 29 Oct 2026)
DF, DA, M_FOIL_CM = 5.169, 2.543, 0.781          # foil drag N, wet-arm drag N, E226 pitching moment Nm (nose-down) at 22.15 kt
REACH, Z_SIDE, H_FOIL, E_LIFT = 0.25, 0.2025, 0.270, 0.005   # m: reach, side-force depth below the pivot, foil depth below the rake pin, lift aft of the pin
T_S = FOIL * REACH          # static cant torque, Nm
T_D = 1.3 * T_S             # design cant torque, Nm
T_UPSET = T_D + F_SIDE * Z_SIDE
T_L = 2.0                   # ASSUMED light raise/lower load (arm + foil weight ~0.4 kg x 0.2 m, plus water drag)
K_UP = T_UPSET / T_D        # every cant-path load scales with the cant torque
BAR_UP_METAL, BAR_UP_CARBON = 1.5, 2.0
print('basis: foil %.2f N, static cant %.2f Nm, design %.2f Nm (was 11.77), upset %.2f Nm (was 18.25); side force %.1f N' % (FOIL, T_S, T_D, T_UPSET, F_SIDE))
rows = []


def chk(part, case, stress, allow, bar=3.0, unit='MPa'):
    m = allow / stress
    rows.append((part, case, '%.0f %s' % (stress, unit), '%.0f' % allow, '%.1fx' % m, 'PASS' if m >= bar else 'FAIL (bar %.1fx)' % bar))


def chk2(part, stress, allow, unit='MPa'):
    """design case (bar 3x) and the upset survival case (bar 1.5x), same load path"""
    chk(part, 'design %.2f Nm' % T_D, stress, allow, 3.0, unit)
    chk(part, 'upset %.1f Nm' % T_UPSET, stress * K_UP, allow, BAR_UP_METAL, unit)


# ---------------- cant drive: HPC M1-50 aluminium-bronze wheel (sector) + hardened 1-start worm, lead 3 deg 48'
z2, m_, r2 = 50, 1.0, 25.0
lam = math.radians(3 + 48 / 60)
mu = 0.10; phi = math.atan(mu); ETA = math.tan(lam) / math.tan(lam + phi)
N0, I0, TST, IST, T_CONT, T_LIM = 230.0, 0.15, 4.0 * KGCM, 2.9, 3.5, 5.0   # Pololu 20D 6V 63:1 CB (#3702)
K_SP, ETA_SP = 2.5, 0.97   # rev E3: m0.5 spur 24T -> 60T (2.5:1) at the same 21 mm centres; rev E2 was 28T -> 56T (2:1)
print('\ncant drive, Pololu 20D 63:1 -> spur 2.5:1 -> worm 50:1 (eta worm %.3f at mu 0.10, spur %.2f); self-locking: %s' % (ETA, ETA_SP, phi > lam))
for name, T, k in (('light raise/lower (assumed)', T_L, K_SP), ('static %.2f Nm' % T_S, T_S, K_SP), ('design %.2f Nm' % T_D, T_D, K_SP),
                   ('rev E2 2:1, static', T_S, 2.0), ('rev E2 2:1, design', T_D, 2.0)):
    Tm = T / (z2 * ETA * k * ETA_SP)
    frac = Tm / TST
    n = N0 * (1 - frac)
    dps = n / k / z2 * 6.0
    I = I0 + (IST - I0) * frac
    print('  %-28s gearmotor %.2f kg.cm (%2.0f%% of stall; continuous %.1f, limit %.1f), %4.0f rpm, cant %.1f deg/s (80 deg in %.1f s), %.1f A' % (
        name, Tm / KGCM, 100 * frac, T_CONT, T_LIM, n, dps, 80 / dps if dps > 0 else float('inf'), I))
Ft = T_D / (r2 / 1000)
Fr = Ft * math.tan(math.radians(20)) / math.cos(lam + phi)
Fw = Ft * math.tan(lam + phi)
print('  mesh at design: wheel tangential / worm axial %.0f N, separating %.0f N, worm tangential %.0f N' % (Ft, Fr, Fw))
b, Y, rp = 10.0, 0.408, 250.0     # face width ASSUMED (HPC to confirm), Lewis factor z=50, CuAl10Ni5Fe4 Rp0.2
chk2('Worm wheel teeth (Lewis, one tooth)', Ft / (b * m_ * Y), rp)
chk2('Worm wheel teeth (Lewis, 2 teeth share)', Ft / (2 * b * m_ * Y), rp)
print('  HPC catalogue rating, M1-50 bronze + hardened worm: 10.06 Nm continuous at 1000 rpm input (static load at the top reach %.2f Nm)' % T_S)
T_in = T_D / (z2 * ETA)
Fs_t = T_in / 0.015                         # tangential at the 60T pitch radius 15 mm
Fs = Fs_t / math.cos(math.radians(20))
chk('Spur gear 60T m0.5 (Lewis, 7075-T6)', 'design', Fs_t / (4 * 0.5 * 0.422), 503)
chk('Spur pinion 24T m0.5 (Lewis, steel C45)', 'design', Fs_t / (4 * 0.5 * 0.337), 340)
print('  spur at design: worm input %.2f Nm, tangential %.0f N; pinion torque on the 4 mm D shaft %.2f Nm' % (T_in, Fs_t, T_in / K_SP / ETA_SP))
R_mesh = math.hypot(Fr, Fw) / 2
R_lo = R_mesh + Fs * 53.0 / 46.0
print('  upset %.1f Nm: worm thrust %.0f N (F8-16M %.0fx); the worm self-locks, so no torque reaches the spur pair or the motor' % (T_UPSET, Ft * K_UP, 7000 / (Ft * K_UP)))
print('  worm thrust bearing F8-16M: %.0f N axial (static ~7 kN, %.0fx); lower 688ZZ radial %.0f N (C0 0.71 kN, %.1fx)' % (Ft, 7000 / Ft, R_lo, 710 / R_lo))

# ---------------- cross shaft 7075-T6, hollow 10/5; sector on two 3 mm hardened pins (rev E3)
SY_AL, TAU_AL = 503.0, 0.577 * 503.0
D, DI = 10.0, 5.0
R, RI = D / 2, DI / 2
J = math.pi / 32 * (D ** 4 - DI ** 4)
Zp = J / R
chk2('Cross shaft torsion, plain (10/5)', T_D * 1000 / Zp, TAU_AL)


def z_net(d_hole):  # polar section modulus of the hollow shaft at a transverse hole (net-section approximation)
    return Zp - d_hole * (D ** 2 - DI ** 2) / 6


chk2('Cross shaft torsion at a 2 mm cradle-pin hole', T_D * 1000 / z_net(2.0), TAU_AL)
chk2('Cross shaft torsion at a 3 mm sector-pin hole', T_D * 1000 / z_net(3.0), TAU_AL)


def p_shaft(T_Nmm, d):  # peak bearing pressure of a cross pin in a hollow shaft, pressure linear with radius
    return 3 * T_Nmm * R / (2 * d * (R ** 3 - RI ** 3))


n_pin, d_pin = 2, 3.0
chk2('Shaft bearing on the sector pins (2 x 3 mm, shared)', p_shaft(T_D * 1000 / n_pin, d_pin), SY_AL)
chk2('Sector pins 2 x 3 mm, shear', (T_D * 1000 / D / n_pin) / (math.pi * d_pin ** 2 / 4), 0.577 * 1000)
s_hub = 4.0      # sector hub wall (18 mm hub on the 10 mm shaft)
chk2('Sector hub bearing on the pins (Al bronze)', T_D * 1000 / (n_pin * d_pin * s_hub * (D + s_hub)), 250)
print('  for the record, the rev E2 single 4 mm pin at %.2f Nm: shaft bearing %.0f MPa (%.1fx, linear model; the uniform model gave %.0f MPa), torsion at its hole %.0f MPa (%.1fx)' % (
    T_D, p_shaft(T_D * 1000, 4.0), SY_AL / p_shaft(T_D * 1000, 4.0), T_D * 1000 / (2 * 3.75) / (4 * 2.5), T_D * 1000 / z_net(4.0), TAU_AL / (T_D * 1000 / z_net(4.0))))
print('  if one 3 mm pin took the whole torque (poor fit): shaft bearing %.0f MPa (%.1fx); so match-drill and ream the pins with the hub on the shaft and set the hub with retaining compound' % (
    p_shaft(T_D * 1000, d_pin), SY_AL / p_shaft(T_D * 1000, d_pin)))

# ---------------- rake pin and bushings: lift couple across the cheeks plus drag (revb_loads.py model)
LD = 1.3 * FOIL; DD = 1.3 * (DF + DA)
for t_ch, lab in ((4.0, 'rev E2 4 mm cheeks'), (5.0, 'rev E3 5 mm cheeks')):
    c_half = 10.0 + t_ch / 2                 # cheek centre from the cant axis, mm (cheeks start 10 mm out)
    s = 2 * c_half; lever = REACH * 1000 + c_half
    R2 = LD * lever / s; Q2 = lever * DD / s
    F_ch = math.hypot(R2, Q2)
    print('  %s: cheek load %.0f N at design (lift couple %.0f N, drag %.0f N)' % (lab, F_ch, R2, Q2))
    if t_ch == 5.0:
        chk2('Rake bushing iglidur G 5x8, 5 mm cheek', F_ch / (5 * t_ch), 80, unit='MPa (p)')
        chk2('Rake pin 5 mm, bending (3.0 mm lever)', F_ch * (t_ch / 2 + 0.5) / (math.pi * 5 ** 3 / 32), 1000)
    else:
        chk('Rake bushing iglidur G 5x8, 4 mm cheek (rev E2)', 'design %.2f Nm' % T_D, F_ch / (5 * t_ch), 80, 3.0, unit='MPa (p)')
chk('Cant bushings iglidur 10 mm', 'design', 80.0 * T_D / 11.77 / (10 * 6), 80, unit='MPa (p)')

# ---------------- fork, Ti-6Al-4V as printed (855 MPa yield)
SY_TI = 855.0
chk2('Fork tongue in the arm cavity (Z 57.8 mm3)', T_D * 1000 / 57.8, SY_TI)
Zb = 20 * 5.1 ** 2 / 6
chk2('Fork bridge at the outboard cheek root', (T_D * 1000 / 24.0) * 21.9 / Zb, SY_TI)

# ---------------- rake moment at the top reach and the rake servo
CL12, CL145 = 36.21 / (0.5 * 999 * (12 * 0.514444) ** 2 * 0.012), 36.21 / (0.5 * 999 * (14.5 * 0.514444) ** 2 * 0.012)
slope = (CL12 - CL145) / (0.70 - 0.20)            # 3D lift slope per deg from the rev D trim table (0.70 deg at 12 kt, 0.20 deg at 14.5 kt)
q = 0.5 * 999 * (22.15 * 0.514444) ** 2
cl = FOIL / (q * 0.012)
aoa = 0.20 - (CL145 - cl) / slope
MOUNT = 2.6
rake = aoa - MOUNT


def m_rake(r_deg, L=FOIL, Df=DF, Da=DA, Mf=M_FOIL_CM):
    return Df * H_FOIL + Da * (H_FOIL - 0.0675) + Mf + L * H_FOIL * math.sin(math.radians(-r_deg)) + L * E_LIFT


M_JOINT = 0.03                 # Nm, the straight-steel joint's thicker strut top (+4.9% arm drag, joint_straight.py)
M_trim = m_rake(rake) + M_JOINT; M_des = 1.3 * M_trim
M_old = 1.3 * m_rake(-4.0, 36.21, 2.66, 1.33, 0.31)
print('\nrake at the top reach: foil CL %.3f, AoA %.2f deg, rake %.2f deg; moment %.2f Nm steady, %.2f Nm design (x1.3); rev E2 design case was %.2f Nm' % (cl, aoa, rake, M_trim, M_des, M_old))
print('  parts of the steady moment: foil drag x 270 mm %.2f, arm drag x 202.5 mm %.2f, foil pitching moment %.2f, lift x rake offset %.2f, lift x 5 mm %.2f Nm' % (
    DF * H_FOIL, DA * (H_FOIL - 0.0675), M_FOIL_CM, FOIL * H_FOIL * math.sin(math.radians(-rake)), FOIL * E_LIFT))
# Rake servo (29 Oct 2026, late): AGF RC A66BHLW, brushless, IP67, 40.8 x 20.2 x 29.5 mm, 66 g, 25T spline.
# Retailer listings (hobbyfactory.fi, rc-modelshop.com; no AGF datasheet found): 25.0 kg.cm / 0.090 s per 60 deg at 6.0 V,
# 30.0 / 0.080 at 7.4 V, 33.0 / 0.068 at 8.4 V (one listing says 0.065). No current figures are listed: measure them.
# It replaces the Hitec DB778WP picked first the same evening (18.0 kg.cm at 7.4 V, $169.99), dropped on cost and
# because it missed the duty rule; the Python 125Mini HV (9.0 kg.cm at 8.4 V) failed the top reach.
SERVO = {6.0: (25.0, 0.090), 7.4: (30.0, 0.080), 8.4: (33.0, 0.068)}
OTHERS = {'Hitec DB778WP': 18.0, 'AGF A50BHLW': 18.0, 'Python 125Mini HV': 7.9}   # kg.cm at 7.4 V, for the record
K_LINK = 24.0 / 4.8              # 5:1, 24 mm fork horn / 4.8 mm servo horn
K_CAD = 25.5 / 4.94              # Hull_v2 motion study: horn +-25.5 deg -> rake +4.92 / -4.96 deg
for nm, M in (('steady', M_trim), ('design', M_des)):
    Ts = M / K_LINK / KGCM
    print('  %s: servo %.2f kg.cm at 5:1; A66BHLW ' % (nm, Ts) + ', '.join(
        '%.1f V %.2fx (%.0f%% of stall)' % (v, t / Ts, 100 * Ts / t) for v, (t, s) in SERVO.items()))
need = max(1.4 * M_des, 3.0 * M_trim) / K_LINK / KGCM
print('  servo needed at 5:1 for 1.4x on the design moment and the steady hold at or under 1/3 of stall (ASSUMED duty rule): '
      '%.1f kg.cm; the A66BHLW gives %.2fx of that at 6.0 V, %.2fx at 7.4 V and %.2fx at 8.4 V' % (
          need, SERVO[6.0][0] / need, SERVO[7.4][0] / need, SERVO[8.4][0] / need))
print('  for the record at 7.4 V: ' + ', '.join('%s %.0f%% of stall steady, %.2fx of the need' % (k, 100 * M_trim / K_LINK / KGCM / t, t / need) for k, t in OTHERS.items()))
Ts = M_trim / K_LINK / KGCM
for v, (t, s) in SERVO.items():
    w0 = 60.0 / s                       # horn deg/s, no load
    wl = w0 * (1 - Ts / t)              # ASSUMED straight-line torque-speed curve
    print('  rake rate at %.1f V: %.0f deg/s unloaded, %.0f deg/s against the steady top-reach moment' % (v, w0 / K_CAD, wl / K_CAD))
F_link = M_des / 0.024
chk('Fork horn root (24 mm horn, link %.0f N)' % F_link, 'design', F_link * 15.5 / (4 * 8 ** 2 / 6), SY_TI)
SY_LINK = 760.0                # 1 mm 301 stainless, half-hard (ASTM A666, yield >= 760 MPa); plain 250 MPa steel gives 2.96x in pin bearing
chk('Link 1 mm 301 half-hard, net section', 'design', F_link / ((4 - 1.5) * 1), SY_LINK)
chk('Link 1 mm 301 half-hard, pin bearing (1.5 x yield)', 'design', F_link / (1.5 * 1.0), 1.5 * SY_LINK)
chk('Link 10.25 mm, Euler buckling (pinned ends)', 'design', F_link, math.pi ** 2 * 200e3 * (4 * 1.0 ** 3 / 12) / 10.25 ** 2, unit='N')
# each link pin joins two layers (horn + link, link + fork horn), so it is in single shear; the rev E2/E3 script said double
chk('Link pins 1.5 mm, single shear', 'design', F_link / (math.pi * 0.75 ** 2), 0.577 * 600)
SY_7075 = 503.0
chk('Servo horn 7075, arm bending at the hub (1.3 mm lever)', 'design', F_link * 1.3 / (1.5 * 5.0 ** 2 / 6), SY_7075)
chk('Servo horn 7075, net section at the pin hole', 'design', F_link / ((5.0 - 1.5) * 1.5), SY_7075)
chk('Servo horn 7075, pin bearing (1.5 x yield)', 'design', F_link / (1.5 * 1.5), 1.5 * SY_7075)
chk('Servo horn 25T spline, hub tooth shear (4 mm on)', 'design', (M_des / K_LINK) / 0.003 / (math.pi * 6.0 * 4.0 * 0.5), 0.577 * SY_7075)
F_stop = M_des / 0.018
chk('Rake stop arm (servo failure, %.0f N)' % F_stop, 'design', F_stop * 8 / (12 * 2.5 ** 2 / 6), SY_TI)

# ---------------- bulkhead: worm-housing flange, 4 x M3 in 4 mm birch ply
chk2('Bulkhead bolt bearing (4 x M3, 4 mm ply)', Ft / 4 / (3.2 * 4), 35)

# ---------------- carbon upper arm root (steel.py rule: E226 arm 40 mm, 20% t/c; Ith from struts2.sect_t; allowable 373 MPa
# for 0/90 plies and 186.5 MPa for +-45 plies, ply-weighted). Root moment = the design cant torque, side force ignored (conservative).
for n, n090, Ith, lab in ((6, 1, 894.5, '6 plies (1 x 0/90), rev E2'), (7, 2, 1043.5, '7 plies with a 0/90 root doubler, rev E3')):
    allow = (373 * n090 + 186.5 * (n - n090)) / n
    sig = T_D * 1000 * (40 * 0.20 / 2) / Ith
    rows.append(('Carbon arm root, ' + lab, 'design %.2f Nm' % T_D, '%.0f MPa' % sig, '%.0f' % allow, '%.2fx' % (allow / sig), 'PASS' if allow / sig >= 4 else 'FAIL (bar 4.0x)'))
    if n == 7:
        rows.append(('Carbon arm root, ' + lab, 'upset %.1f Nm' % T_UPSET, '%.0f MPa' % (sig * K_UP), '%.0f' % allow, '%.2fx' % (allow / sig / K_UP), 'PASS' if allow / sig / K_UP >= BAR_UP_CARBON else 'FAIL (bar 2.0x)'))
x_need = 250 * (4.13 * 11.77 / T_D) / 4.0
print('\ncarbon arm, 6 plies: clears 4x where the lift lever is under %.0f mm, i.e. from about %.0f mm (horizontal) out from the pivot; the doubler runs from the arm end to 60 mm along the arm and tapers out by 75 mm' % (x_need, 250 - x_need))

print('\n%-52s %-18s %-12s %-6s %-7s %s' % ('Part', 'Case', 'Stress', 'Allow', 'Margin', 'Result'))
for r in rows:
    print('%-52s %-18s %-12s %-6s %-7s %s' % r)
