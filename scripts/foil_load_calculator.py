"""
Foil hydrodynamic load calculator.

This is a runnable implementation of the method in the "Hydrodynamic Load
Estimation" doc (Mechanisms folder, project Google Drive): the lift
equation, Reynolds number, center-of-pressure / cant-torque calc, a
rudder side-load estimate, a possible-drag-range estimate (profile +
induced), and a helper to turn a real DIY tow-test reading into an
empirical CL -- wired together so you can plug in real numbers as they
firm up instead of redoing the algebra by hand each time.

IMPORTANT LIMITATION, stated plainly: this sandbox has no network access
to install a real 2D panel-method / low-Re airfoil solver (XFOIL,
NeuralFoil, etc. were all unreachable when checked). So CL(alpha) here is
either:
  (a) a number YOU supply from real section data (best -- e.g. a published
      low-Re polar for your actual section, or a number corrected against
      your own tow test via measured_cl() below), or
  (b) a coarse thin-airfoil-theory fallback (CL = 2*pi*alpha) with a crude
      Reynolds-number derate, provided so the tool still runs with nothing
      but a chord/speed guess.
Treat (b) as a placeholder to get the tool moving, not a validated
prediction -- see the Hydrodynamic Load Estimation doc, sections 3 and 8,
for why low-Re CL is genuinely uncertain at this scale and how to ground
it with a cheap physical test.

Run: python3 foil_load_calculator.py
Requires: nothing beyond the standard library + the numbers below.
"""

import math

RHO_FRESHWATER = 999.0   # kg/m^3 at ~20C -- use this for a lake
RHO_SEAWATER = 1025.0
NU_WATER = 1.0e-6        # kinematic viscosity of water, m^2/s at ~20C
G = 9.81


def reynolds_number(speed_mps, chord_m, nu=NU_WATER):
    """Re = V*c/nu. RC-scale foils typically land around 50,000-200,000 --
    see the Hydrodynamic Load Estimation doc, section 3, for why that
    matters."""
    return speed_mps * chord_m / nu


def cl_thin_airfoil(alpha_deg, re=None, re_full_confidence=150_000, re_floor=50_000):
    """Coarse CL(alpha) fallback: inviscid thin-airfoil theory
    (CL = 2*pi*alpha) with a simple linear derate below re_full_confidence,
    reaching 70% of the inviscid value at re_floor. This is illustrative,
    not a validated low-Re model -- swap in real section data (or a
    tow-test-corrected number, via measured_cl()) as soon as you have one.
    See doc section 3.
    """
    alpha_rad = math.radians(alpha_deg)
    cl = 2 * math.pi * alpha_rad
    if re is not None and re < re_full_confidence:
        span = re_full_confidence - re_floor
        frac = max(0.0, min(1.0, (re - re_floor) / span)) if span > 0 else 1.0
        derate = 0.7 + 0.3 * frac
        cl *= derate
    return cl


def lift_force(speed_mps, area_m2, cl, rho=RHO_FRESHWATER):
    """L = 0.5 * rho * V^2 * S * CL, in Newtons."""
    return 0.5 * rho * speed_mps ** 2 * area_m2 * cl


def required_area(weight_n, speed_mps, cl, rho=RHO_FRESHWATER):
    """Solve the lift equation for foil area given a target weight/speed/CL."""
    return weight_n / (0.5 * rho * speed_mps ** 2 * cl)


def required_speed(weight_n, area_m2, cl, rho=RHO_FRESHWATER):
    """Solve the lift equation for speed given a target weight/area/CL."""
    return math.sqrt(weight_n / (0.5 * rho * area_m2 * cl))


def quarter_chord_cp(chord_m):
    """Standard aerodynamic-center approximation for thin sections at
    modest AoA. See doc section 4 for the caveats."""
    return 0.25 * chord_m


def cant_torque(lift_n, lever_arm_m):
    """Torque about the cant pivot axis = lift force x perpendicular
    distance from the center of pressure to the pivot axis.

    IMPORTANT CAVEAT, not just a formatting note: this is the STATIC
    holding torque at a steady trim condition (lift == weight supported).
    Because trim lift always equals weight in steady, level foiling
    regardless of speed, this number comes out nearly identical at takeoff
    and cruise -- that's a real property of the physics, not a bug in this
    calc, but it means this number is a lower bound, not the whole story.

    The reference build's 20+ Nm failure was very likely dominated by
    DYNAMIC/maneuvering torque -- physically rotating a still-loaded,
    still-lifting arm through an arc during a tack/gybe while flow keeps
    hitting it -- which this simplified static calc does not capture and
    which is genuinely hard to hand-calculate. Treat this function's
    output as a floor to size the safety margin (doc section 7) from, and
    lean on the DIY tow test (doc section 8) -- ideally extended to
    include swinging the test arm through its cant arc under load with a
    torque wrench or load cell on the lever -- to close that gap before
    finalizing an actuator.
    """
    return lift_n * lever_arm_m


def kt_to_mps(knots):
    return knots * 0.514444


def mps_to_kt(mps):
    return mps / 0.514444


def n_to_kgf(newtons):
    return newtons / G


def measured_cl(lift_n, speed_mps, area_m2, rho=RHO_FRESHWATER):
    """Back out an empirical CL from a real tow-test measurement -- the
    inverse of lift_force(). Feed it a lift reading (N, from the vertical
    support-line scale), the speed you towed at (m/s), and the test
    article's actual planform area (m^2, measured directly off the
    physical piece, not assumed).

    IMPORTANT: lift_n must already be baseline-corrected -- it is the
    towing reading MINUS the at-rest reading on that same vertical line
    (the assembly's own net submerged weight, which is not lift and will
    not generally be zero). See the Hydrodynamic Load Estimation doc,
    section 8, for why.

    Compare the result against cl_thin_airfoil() at the same AoA/Re to see
    how far off the fallback model was for your real section, and use the
    measured number in place of the fallback for that AoA/Re going
    forward."""
    return lift_n / (0.5 * rho * speed_mps ** 2 * area_m2)


# ----------------------------------------------------------------------------
# Drag -- profile + induced only. This is a FLOOR, not a full prediction:
# it deliberately excludes wave-making drag (see drag_force_range()'s
# docstring), which can be a real, sometimes dominant contributor for a
# lifting surface running near the free surface, and which nobody has a
# clean hand-calc for. Use this range to sanity-check the tow test's
# horizontal-line reading, not to replace it.
# ----------------------------------------------------------------------------

def skin_friction_coefficient(re, flow="turbulent"):
    """Flat-plate skin friction coefficient, one side.
    'laminar'   -> Blasius: Cf = 1.328 / sqrt(Re)
    'turbulent' -> Schlichting: Cf = 0.455 / (log10(Re))**2.58
    Real flow at RC-scale Re is probably a mix (partial laminar run with a
    transition point that depends on section/surface finish) -- treat
    laminar and turbulent as the two ends of a plausible range, not a
    choice to resolve analytically."""
    if flow == "laminar":
        return 1.328 / math.sqrt(re)
    elif flow == "turbulent":
        return 0.455 / (math.log10(re) ** 2.58)
    else:
        raise ValueError("flow must be 'laminar' or 'turbulent'")


def profile_drag_coefficient(re, thickness_ratio=0.08, flow="turbulent"):
    """2D section profile (parasite) drag coefficient: two-sided skin
    friction, corrected for thickness with a standard subsonic form-factor
    approximation carried over from aircraft wing drag build-up methods
    (e.g. Raymer-style): FF = 1 + 2*(t/c) + 60*(t/c)**4. thickness_ratio
    is the section's max thickness / chord (0.08 = 8%, a reasonable thin
    hydrofoil-section starting guess -- use your actual section's value
    once chosen)."""
    cf = skin_friction_coefficient(re, flow=flow)
    form_factor = 1 + 2 * thickness_ratio + 60 * thickness_ratio ** 4
    return 2 * cf * form_factor  # x2: wetted area is both sides of the section


def aspect_ratio_from_area_chord(area_m2, chord_m):
    """AR = span^2 / area, assuming a rectangular planform (span = area/chord).
    A real tapered/swept planform will differ -- treat as a first-order
    estimate, consistent with the rest of this tool's simplifications."""
    span_m = area_m2 / chord_m
    return span_m ** 2 / area_m2


def induced_drag_coefficient(cl, aspect_ratio, oswald_e=0.85):
    """Lift-induced drag: CDi = CL^2 / (pi * AR * e). oswald_e (span
    efficiency) of 0.85 is a reasonable well-loaded planform; use 0.7 for
    a more conservative (less efficient) assumption -- see
    drag_force_range()."""
    return cl ** 2 / (math.pi * aspect_ratio * oswald_e)


def drag_force(speed_mps, area_m2, cd, rho=RHO_FRESHWATER):
    """D = 0.5 * rho * V^2 * S * CD, in Newtons -- same form as lift_force()."""
    return 0.5 * rho * speed_mps ** 2 * area_m2 * cd


def drag_force_range(speed_mps, area_m2, chord_m, cl, thickness_ratio=0.08, rho=RHO_FRESHWATER):
    """A plausible LOW-HIGH drag range from profile + induced drag only.

    NOT INCLUDED, and this matters: wave-making drag. A lifting surface
    running near the free surface generates its own wave pattern, adding
    drag that depends on submergence depth and Froude number in a way
    that doesn't reduce to a simple formula here -- it's small when the
    foil runs several chords deep and can be a real, sometimes dominant
    contributor at shallow submergence or near hump/takeoff speed. There
    is no credible hand-calc for it included in this tool; the tow test's
    horizontal-line reading (Hydrodynamic Load Estimation doc, section 8)
    is the practical way to see it. If your measured drag comes in well
    above this range, wave drag (or strut/mount interference drag) is a
    good first place to suspect why.

    Returns (low_n, high_n): low = laminar skin friction + oswald_e=0.85,
    high = turbulent skin friction + oswald_e=0.70. Reality is probably
    somewhere inside this bracket, not at either end.
    """
    re = reynolds_number(speed_mps, chord_m)
    ar = aspect_ratio_from_area_chord(area_m2, chord_m)

    cd_profile_lo = profile_drag_coefficient(re, thickness_ratio, flow="laminar")
    cd_profile_hi = profile_drag_coefficient(re, thickness_ratio, flow="turbulent")
    cd_induced_lo = induced_drag_coefficient(cl, ar, oswald_e=0.85)
    cd_induced_hi = induced_drag_coefficient(cl, ar, oswald_e=0.70)

    low_n = drag_force(speed_mps, area_m2, cd_profile_lo + cd_induced_lo, rho)
    high_n = drag_force(speed_mps, area_m2, cd_profile_hi + cd_induced_hi, rho)
    return low_n, high_n


if __name__ == "__main__":
    print("=" * 70)
    print("WORKED EXAMPLE -- placeholder numbers, replace with real ones")
    print("=" * 70)

    # ---- Placeholder inputs: replace these with real numbers as they firm up ----
    total_weight_kg = 3.5          # PLACEHOLDER -- Weight Budget (Design & Specs sec.9) is still empty
    n_main_foils = 2                # port + starboard
    chord_m = 0.05                  # PLACEHOLDER foil chord, 5cm
    lever_arm_m = 0.06              # PLACEHOLDER: quarter-chord CP to cant pivot distance
    takeoff_speed_kt = 4.3           # bracket from Moth/WASZP precedent, doc section 2
    cruise_speed_kt = 8.0            # PLACEHOLDER cruise target once foiling and powered up
    design_aoa_deg = 4.0              # modest working AoA, per doc section 3 (0.3-0.5 CL bracket)
    safety_margin = 2.5               # per doc section 7

    weight_n = total_weight_kg * G
    weight_per_foil_n = weight_n / n_main_foils   # first pass: ignore rudder's trim share

    print(f"\nTotal weight (placeholder): {total_weight_kg} kg = {weight_n:.1f} N")
    print(f"Per main foil (of {n_main_foils}): {weight_per_foil_n:.1f} N")

    for label, speed_kt in [("TAKEOFF", takeoff_speed_kt), ("CRUISE", cruise_speed_kt)]:
        v = kt_to_mps(speed_kt)
        re = reynolds_number(v, chord_m)
        cl = cl_thin_airfoil(design_aoa_deg, re=re)
        area_needed = required_area(weight_per_foil_n, v, cl)
        lift_at_area = lift_force(v, area_needed, cl)  # sanity check, should equal weight_per_foil_n
        torque = cant_torque(lift_at_area, lever_arm_m)
        torque_margined = torque * safety_margin

        print(f"\n-- {label}: {speed_kt} kt ({v:.2f} m/s) --")
        print(f"  Reynolds number:            {re:,.0f}")
        print(f"  CL at {design_aoa_deg} deg AoA (derated): {cl:.3f}")
        print(f"  Required foil area (1 side): {area_needed*1e4:.1f} cm^2  ({area_needed:.4f} m^2)")
        print(f"  Cant-pivot torque:            {torque:.3f} N*m")
        print(f"  With {safety_margin}x safety margin:    {torque_margined:.3f} N*m")

    print("\n" + "=" * 70)
    print("RUDDER SIDE-LOAD -- separate calc, do not reuse the main-foil torque")
    print("=" * 70)
    rudder_area_m2 = 0.0015   # PLACEHOLDER, smaller than a main foil
    rudder_chord_m = 0.035
    rudder_deflect_deg = 8.0  # a representative steering deflection
    v = kt_to_mps(cruise_speed_kt)
    re_r = reynolds_number(v, rudder_chord_m)
    cl_r = cl_thin_airfoil(rudder_deflect_deg, re=re_r)
    side_force = lift_force(v, rudder_area_m2, cl_r)
    print(f"Rudder side force at {rudder_deflect_deg} deg deflection, {cruise_speed_kt}kt cruise: "
          f"{side_force:.2f} N ({n_to_kgf(side_force):.2f} kgf)")
    print("This loads the steering servo/post, NOT the rake actuator -- run the same")
    print("required_area()/cant_torque() calls above with the rudder's own area/chord")
    print("for its separate rake-axis torque.")

    print("\nReminder: replace every PLACEHOLDER above with real numbers as soon as")
    print("you have them, and cross-check with the DIY tow test (doc section 8)")
    print("before trusting this for a final actuator spec.")
    print("\nNote the cant torque came out ~identical at takeoff and cruise -- that's")
    print("expected (trim lift always equals weight), not a bug. See cant_torque()'s")
    print("docstring: this is a STATIC lower bound, not the dynamic/maneuvering torque")
    print("that likely drove the reference build's 20+ Nm underestimate.")

    print("\n" + "=" * 70)
    print("EXAMPLE: turning a tow-test reading into an empirical CL")
    print("=" * 70)
    example_lift_reading_n = 2.1   # PLACEHOLDER -- your vertical-scale reading, already baseline-corrected
    example_speed_mps = 2.0        # PLACEHOLDER -- your measured tow speed
    example_area_m2 = 0.006        # PLACEHOLDER -- your test piece's actual planform area
    cl_meas = measured_cl(example_lift_reading_n, example_speed_mps, example_area_m2)
    print(f"Example: {example_lift_reading_n}N lift at {example_speed_mps} m/s over "
          f"{example_area_m2*1e4:.0f} cm^2 -> measured CL = {cl_meas:.3f}")
    print("Compare that against cl_thin_airfoil() at the same AoA/Re -- the gap between")
    print("them is exactly what the fallback model was getting wrong for your section.")

    print("\n" + "=" * 70)
    print("POSSIBLE DRAG RANGE -- profile + induced only, NOT wave drag (see docstring)")
    print("=" * 70)
    for label, speed_kt in [("TAKEOFF", takeoff_speed_kt), ("CRUISE", cruise_speed_kt)]:
        v = kt_to_mps(speed_kt)
        re = reynolds_number(v, chord_m)
        cl = cl_thin_airfoil(design_aoa_deg, re=re)
        area_needed = required_area(weight_per_foil_n, v, cl)  # same sizing as the lift example
        lo_n, hi_n = drag_force_range(v, area_needed, chord_m, cl)
        print(f"\n-- {label}: {speed_kt} kt ({v:.2f} m/s), area {area_needed*1e4:.1f} cm^2 --")
        print(f"  Possible drag range (this foil, profile+induced only): "
              f"{lo_n:.2f} - {hi_n:.2f} N ({n_to_kgf(lo_n):.2f} - {n_to_kgf(hi_n):.2f} kgf)")
    print("\nReal drag (from the tow test's horizontal line) should land inside or not far")
    print("above this range. Meaningfully above it points at wave drag or strut/mount")
    print("interference, not at an error in this calc.")
