"""Foilarm_v4 strength check (2 Oct 2026, Foilarm_v4 CAD as saved the same day).
Part A re-runs steel.py's hybrid() as before (global arm model). Needs steel.py, struts2.py, opt.py, arm.py, arm_pts.csv in the same folder.
Part B checks the v4-specific local features with section properties measured in Fusion (Foilarm_v4, saved 2 Oct 2026).
All load cases in part B are ASSUMPTIONS (marked) and should be confirmed by the builder; capacities are given so any other
load case can be read straight off.
"""
import io, contextlib, json, math
import numpy as np
with contextlib.redirect_stdout(io.StringIO()):
    import steel
out = {}

# ---------------- Part A: steel.py baseline (unchanged model) ----------------
steel.Wetted = 135.0
h = steel.hybrid(22, 0.18, 135, 40, 0.20, 6, 2.8, 0.25, steel.MAT['17-4PH H1025'])
out['A_steel_py'] = {k: round(float(h[k]), 3) for k in ('Mj', 'sig_s', 'marg_s', 'marg_c', 'tw', 'tw_s', 'tw_c', 'Ith', 'xoff')}

# ---------------- constants ----------------
g = 9.81; rho = 1025.0; kt = 0.514444
SY = 1000.0                 # 17-4PH H1025 yield, MPa (doc value, typical, not certified)
TAU_Y = 0.58 * SY
BAR = 3.0                   # project bar on yield for steel
L_des = 36.21               # N, foil lift (weight + rudder downforce), Design & Specs section 3
KL = 1.30                   # load factor used for the cant torque (Design & Specs)
T_wire_worst = 0.148 / 4.193e-3   # N: 148 mNm worst-case pivot moment on the 4.193 mm lever (Mechanisms 2c)
P_lift = KL * L_des
T_des = KL * T_wire_worst
out['loads'] = dict(L_des=L_des, lift_x1p3=round(P_lift, 2), wire_worst=round(T_wire_worst, 2), wire_x1p3=round(T_des, 2))

# ---------------- B1: pivot pin, tang, slot walls (lift path) ----------------
P_pin = P_lift + T_des      # lift pushes the foil up into the strut, wire pulls the lug up: the pin reacts both
d_pin = 1.2; t_tang = 1.2; slot_w = 1.4; wall = 1.28
A_pin = math.pi / 4 * d_pin**2
tau_pin = P_pin / (2 * A_pin)
br_tang = P_pin / (d_pin * t_tang)
br_wall = (P_pin / 2) / (d_pin * wall)
# pin bending: load P over the tang width, reactions P/2 over each wall width
a_w = slot_w / 2 + wall / 2            # wall centre from mid-plane
M_pin = (P_pin / 2) * a_w - (P_pin / 2) * (t_tang / 4)
Z_pin = math.pi * d_pin**3 / 32
sig_pin = M_pin / Z_pin
out['B1_pin'] = dict(P_pin_N=round(P_pin, 1), tau_MPa=round(tau_pin, 1), tau_margin_on_shear_yield=round(TAU_Y / tau_pin, 1),
                     bearing_tang_MPa=round(br_tang, 1), bearing_wall_MPa=round(br_wall, 1),
                     pin_bending_MPa=round(sig_pin, 0), pin_yield_needed_for_3x_MPa=round(BAR * sig_pin, 0),
                     pin_margin_if_17_4PH_H1025=round(SY / sig_pin, 2), pin_margin_if_A2_stainless_210MPa=round(210 / sig_pin, 2))

# ---------------- B2: roll and yaw through the tang (measured CAD: neck at z=2.6-3.1 mm above chord line) ----------------
A_neck = 5.32; Iy_neck = 0.638; cx_neck = -0.066; xhalf = 0.57          # mm2, mm4, mm, mm (Fusion, 2 Oct 2026)
S_neck = Iy_neck / (xhalf + abs(cx_neck))
sig_ax_neck = P_pin / A_neck
M_roll_yield = (SY - sig_ax_neck) * S_neck / 1e3            # Nm, first yield
M_roll_3x = (SY / BAR - sig_ax_neck) * S_neck / 1e3
# slot wall as a cantilever from the slot top, loaded at the slot mouth by the tilting tang
z_top_slot = 7.25; z_mouth = 2.5; z_tang_top = 6.33
arm_couple = z_tang_top - z_mouth                           # contact couple arm on the tang
arm_wall = z_top_slot - z_mouth
wall_cap = {}
for b_eff in (5.7, 8.0):
    S_w = b_eff * wall**2 / 6
    sig_per_Nm = (1e3 / arm_couple) * arm_wall / S_w        # MPa per Nm of roll moment
    wall_cap[b_eff] = round(SY / sig_per_Nm, 2)
out['B2_roll_capacity'] = dict(S_neck_mm3=round(S_neck, 3), tang_neck_yield_Nm=round(M_roll_yield, 2), tang_neck_3x_Nm=round(M_roll_3x, 3),
                               slot_wall_yield_Nm_by_effective_width=wall_cap,
                               CP_shift_at_yield_mm=round(M_roll_yield / L_des * 1e3, 1), CP_shift_at_3x_mm=round(M_roll_3x / L_des * 1e3, 1))
cases = {
    'R0 centre of pressure 10 mm off the strut axis (about 7% of the half-span)': L_des * 0.010,
    'R1 outer quarter of one half-span loses all lift (tip ventilation)': L_des * 0.125 * (0.875 * 0.150),
    'R2 one whole half-span loses all lift (half-foil stall or ventilation)': L_des * 0.5 * 0.075,
}
roll = {}
for k, M in cases.items():
    Md = KL * M
    roll[k] = dict(M_Nm=round(M, 3), M_x1p3=round(Md, 3), margin_on_yield=round(M_roll_yield / Md, 2), meets_3x=bool(M_roll_yield / Md >= BAR))
out['B2_roll_cases'] = roll
# yaw: torsion of the 3.4 mm x 1.14 mm tang plate, alpha=0.267 for a/b ~3
a_t, b_t = 3.4, 1.14
T_yaw_yield = TAU_Y * 0.267 * a_t * b_t**2 / 1e3
D_foil = 2.0     # N, foil drag at 10 kt (assumed round-up of the E226 figure)
yaw_cases = {'foil drag 10% unequal across the span': D_foil * 0.10 * 0.075, 'outer quarter of one half loses its drag share': D_foil * 0.125 * 0.131}
out['B2_yaw'] = dict(tang_torsion_yield_Nm=round(T_yaw_yield, 3), cases={k: dict(M_Nm=round(v, 3), margin=round(T_yaw_yield / (KL * v), 1)) for k, v in yaw_cases.items()})

# ---------------- B3: wire lug and wire ----------------
t_lug = 0.5; lig = 0.4; Kt_lug = 3.0
sig_lug = T_des / (2 * lig * t_lug)
out['B3_lug'] = dict(lug_thickness_mm=t_lug, ligament_mm=lig, net_avg_MPa=round(sig_lug, 0), with_Kt3_MPa=round(sig_lug * Kt_lug, 0), margin_on_yield=round(SY / (sig_lug * Kt_lug), 2),
                     bearing_wire_on_lug_MPa=round(T_des / (0.8 * t_lug), 0))
A_wire = math.pi / 4 * 0.8**2
out['B3_wire'] = dict(stress_MPa=round(T_des / A_wire, 0), UTS_needed_for_5x_MPa=round(5 * T_des / A_wire, 0))

# ---------------- B4: strut side force and the spigot joint (measured CAD sections) ----------------
c_s = 0.022; L_w = 0.135; S_s = c_s * L_w
AR = 2 * L_w / c_s                                   # free surface doubles the aspect ratio
a_s = 2 * math.pi * AR / (AR + 2)                    # per rad
Iy_strut = 47.114; x_strut = 1.95                    # z=1..10.2 cm, bore on the neutral axis
Iy_tab = 5.85; x_tab = 1.40                          # z=10.8..13.4 cm: tab + 2.8 mm boss - 1.4 mm bore
Iy_sleeve = 41.48; x_sleeve = 1.95                   # carbon sleeve (measured at z=11.5 and 13.4)
Es = 197e3
Ec_lo, Ec_hi = 15.6e3, 50e3
Ec = 50e3 / 6 + 15.6e3 * 5 / 6                       # steel.py laminate rule, 6 plies
allow_c = 373 * (1 / 6) + 373 * 0.5 * (5 / 6)        # steel.py carbon allowable, MPa
b_tab, l_tab, arm_tab = 12.0, 30.0, 1.4             # tab chord (mm), bonded length (mm), tension-compression couple arm in the tab (mm)
tau_epoxy_wet = 15.0                                 # MPa, assumed wet/aged epoxy lap-shear allowable (not tested)
peak = 2.5                                           # shear-lag peak/average factor at the tab ends (assumed)
rows = []
for V_kt in (10.0, 14.5):
    q = 0.5 * rho * (V_kt * kt) ** 2
    for alpha in (1, 2, 3, 4):
        F = q * S_s * a_s * math.radians(alpha)
        M = KL * F * (L_w / 2) * 1e3                   # N mm at the joint (z = 135 mm), uniform load over the wet length
        M_below = M * (0.105 / 0.135) ** 2             # at z = 105 mm (top of the full-thickness strut)
        sig_strut = M_below * x_strut / Iy_strut
        EI = Es * Iy_tab + Ec * Iy_sleeve
        sig_tab = M * x_tab * Es / EI
        sig_tab_hi = M * x_tab * Es / (Es * Iy_tab + Ec_lo * Iy_sleeve)    # carbon at its softest: steel takes most
        sig_c = M * x_sleeve * Ec / EI
        M_s = M * Es * Iy_tab / EI
        tau_avg = (M_s / arm_tab) / (b_tab * l_tab)
        rows.append(dict(V_kt=V_kt, leeway_deg=alpha, F_side_N=round(F, 1), M_joint_Nm=round(M / 1e3, 2),
                         steel_strut_MPa=round(sig_strut, 0), steel_strut_margin=round(SY / sig_strut, 1),
                         tab_MPa=round(sig_tab, 0), tab_margin=round(SY / sig_tab, 1), tab_margin_soft_carbon=round(SY / sig_tab_hi, 1),
                         tab_alone_margin=round(SY / (M * x_tab / Iy_tab), 1),
                         carbon_MPa=round(sig_c, 0), carbon_margin=round(allow_c / sig_c, 1),
                         bond_tau_avg=round(tau_avg, 1), bond_tau_peak=round(tau_avg * peak, 1), bond_margin=round(tau_epoxy_wet / (tau_avg * peak), 1)))
out['B4_side_force_rows'] = rows
out['B4_const'] = dict(AR_eff=round(AR, 1), CL_alpha_per_rad=round(a_s, 2), E_carbon_6ply_GPa=round(Ec / 1e3, 1), allow_carbon_MPa=round(allow_c, 0),
                       steel_share_of_moment=round(Es * Iy_tab / (Es * Iy_tab + Ec * Iy_sleeve), 2))
# capacity form: strut side force at which each limit is reached (at the joint, with the 1.3 factor)
EI = Es * Iy_tab + Ec * Iy_sleeve
Mcap = {
    'steel strut at z=105 mm yields': SY * Iy_strut / x_strut / (0.105 / 0.135) ** 2,
    'steel tab/boss yields': SY * EI / (Es * x_tab),
    'steel tab/boss reaches the 3x bar': SY / BAR * EI / (Es * x_tab),
    'carbon sleeve reaches its allowable': allow_c * EI / (Ec * x_sleeve),
    'bond reaches 3x (15 MPa wet, peak 2.5x)': (tau_epoxy_wet / BAR / peak) * (b_tab * l_tab) * arm_tab * EI / (Es * Iy_tab),
}
cap = {}
for k, M in Mcap.items():
    Fcap = M / KL / (L_w / 2 * 1e3)
    cap[k] = dict(M_joint_Nm=round(M / 1e3, 2), side_force_N=round(Fcap, 1),
                  leeway_deg_at_10kt=round(math.degrees(Fcap / (0.5 * rho * (10 * kt) ** 2 * S_s * a_s)), 1),
                  leeway_deg_at_14p5kt=round(math.degrees(Fcap / (0.5 * rho * (14.5 * kt) ** 2 * S_s * a_s)), 1))
out['B4_capacity'] = cap
# drag-direction bending at the joint (strong axis)
Ix_tab = 206.6; y_tab = 6.9
M_drag = KL * (D_foil * 135 + 0.79 * 67.5)
out['B4_drag'] = dict(M_Nm=round(M_drag / 1e3, 3), tab_MPa=round(M_drag * y_tab / Ix_tab, 1), margin=round(SY / (M_drag * y_tab / Ix_tab), 0))
# nominal case in the docs
out['B4_doc_nominal'] = dict(M_joint_Nm=round(float(h['Mj']), 3), note='lift-offset term only (1 mm of horizontal offset); no strut side force')
print(json.dumps(out, indent=1))
json.dump(out, open('strength_v4_out.json', 'w'), indent=1)
