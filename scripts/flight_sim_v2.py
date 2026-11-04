"""
Flight-control simulator v2 -- physics-grounded plant.

pid_simulator.py (v1, delivered earlier via chat) used a generic 2nd-order
spring/damper as a stand-in plant, useful for basic PID intuition but not
tied to this boat's actual numbers. This version replaces that plant with
one built from the real lift equation in foil_load_calculator.py: mass,
foil area, boat speed, and CL(alpha) drive the vertical (heave) force, and
Newton's second law integrates that into height error over time.

Still a simplification -- stated plainly, not hidden:
  - Boat speed is held constant. Real speed couples with pitch/heave and
    wind; modeling that properly is a separate, bigger VPP-style project,
    not needed to get useful PID-tuning intuition here.
  - Added mass and heave damping (added_mass_kg, damping_ns_per_m below)
    are placeholder constants. There's no clean formula for these without
    real wetted hull/foil geometry -- treat the settle-time numbers here
    as relative (compare gain sets to each other), not as a promise of
    real-world seconds, until corrected against actual on-water behavior.
  - CL(alpha) reuses foil_load_calculator's thin-airfoil-theory + Re-derate
    placeholder (no low-Re polar tool was reachable from this sandbox --
    see that file's docstring). Swap in real section data as soon as you
    have it and rerun.

This is a tool for exploring PID gain behavior against more realistic
physics than v1 -- still not a substitute for the bench test and the DIY
tow test in the Hydrodynamic Load Estimation doc.

Run: python3 flight_sim_v2.py  (needs foil_load_calculator.py in the same folder)
Requires: matplotlib
"""

import os
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from foil_load_calculator import (
    RHO_FRESHWATER, G, kt_to_mps,
    reynolds_number, cl_thin_airfoil, lift_force,
)


class PID:
    """Same structure as pid_simulator.py's PID and the Arduino sketch's
    pidUpdate() -- kept 1:1 on purpose so tuning intuition transfers."""

    def __init__(self, kp, ki, kd, output_min, output_max, integral_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_min, self.output_max = output_min, output_max
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.prev_error = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def update(self, setpoint, measurement, dt):
        error = setpoint - measurement
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        derivative = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(self.output_min, min(self.output_max, output))


class HeavePlant:
    """
    1-DOF heave dynamics from the real lift equation:

        m_eff * d2z/dt2 = Lift(V, S, CL(alpha_trim + flap_deg)) - W
                            + gust_n(t) - c * dz/dt

    z is height error from target ride height (m); positive z = too high.
    Flap deflection trims AoA around the speed's trim AoA (alpha_trim,
    solved so lift == weight at that speed with zero flap input).
    """

    def __init__(self, mass_kg, area_m2, speed_mps, chord_m=0.05,
                 added_mass_kg=0.4, damping_ns_per_m=9.0,
                 flap_limit_deg=10.0, rho=RHO_FRESHWATER):
        self.weight_n = mass_kg * G
        self.m_eff = mass_kg + added_mass_kg  # PLACEHOLDER added mass, see module docstring
        self.S = area_m2
        self.V = speed_mps
        self.chord = chord_m
        self.c = damping_ns_per_m
        self.flap_limit = flap_limit_deg
        self.rho = rho
        self.re = reynolds_number(speed_mps, chord_m)
        self.alpha_trim = self._solve_trim_aoa()

        self.z = 0.0
        self.vz = 0.0

    def _solve_trim_aoa(self):
        """Bisect for the AoA where lift == weight at this plant's speed."""
        lo, hi = 0.0, 15.0
        for _ in range(80):
            mid = (lo + hi) / 2
            cl = cl_thin_airfoil(mid, re=self.re)
            lift = lift_force(self.V, self.S, cl, self.rho)
            if lift < self.weight_n:
                lo = mid
            else:
                hi = mid
        return (lo + hi) / 2

    def step(self, flap_deg, dt, gust_n=0.0):
        flap_deg = max(-self.flap_limit, min(self.flap_limit, flap_deg))
        alpha = self.alpha_trim + flap_deg
        alpha = max(0.0, alpha)  # can't usefully go negative in this simple model
        cl = cl_thin_airfoil(alpha, re=self.re)
        lift = lift_force(self.V, self.S, cl, self.rho)

        net_force = (lift - self.weight_n) + gust_n - self.c * self.vz
        az = net_force / self.m_eff
        self.vz += az * dt
        self.z += self.vz * dt
        return self.z


def run_sim(plant_kwargs, kp, ki, kd, duration=4.0, dt=0.02,
            initial_z_offset=0.03, gust_at=2.0, gust_n=-6.0,
            flap_limit_deg=10.0):
    plant = HeavePlant(flap_limit_deg=flap_limit_deg, **plant_kwargs)
    plant.z = initial_z_offset  # simulate starting off-target (e.g. a gust just lifted the boat)

    pid = PID(kp, ki, kd, output_min=-flap_limit_deg, output_max=flap_limit_deg,
              integral_limit=flap_limit_deg * 2)

    t, z_log, flap_log = [], [], []
    steps = int(duration / dt)
    for i in range(steps):
        now = i * dt
        g = gust_n if abs(now - gust_at) < dt / 2 else 0.0
        flap = pid.update(setpoint=0.0, measurement=plant.z, dt=dt)
        plant.step(flap, dt, gust_n=g)
        t.append(now)
        z_log.append(plant.z * 100)  # cm, easier to read
        flap_log.append(flap)

    return t, z_log, flap_log, plant.alpha_trim


if __name__ == "__main__":
    # Foil area sized at takeoff per foil_load_calculator.py's worked example
    # (2 foils, 3.5kg placeholder weight, 4.3kt takeoff, 4deg design AoA).
    weight_kg = 3.5
    per_foil_area_m2 = 0.0182  # from foil_load_calculator.py's takeoff sizing
    cruise_v = kt_to_mps(8.0)

    plant_kwargs = dict(
        mass_kg=weight_kg / 2,   # this plant models ONE foil's share of the boat
        area_m2=per_foil_area_m2,
        speed_mps=cruise_v,
    )

    trials = [
        ("Too soft  (Kp=1.5, Ki=0.3, Kd=0.1)", 1.5, 0.3, 0.1),
        ("Starting point (Kp=6.0, Ki=1.5, Kd=0.8)", 6.0, 1.5, 0.8),
        ("Too aggressive (Kp=16.0, Ki=5.0, Kd=1.2)", 16.0, 5.0, 1.2),
    ]

    fig, axes = plt.subplots(len(trials), 2, figsize=(11, 9), sharex=True)

    alpha_trim_reported = None
    for row, (label, kp, ki, kd) in enumerate(trials):
        t, z_log, flap_log, alpha_trim = run_sim(plant_kwargs, kp, ki, kd)
        alpha_trim_reported = alpha_trim

        ax_z, ax_flap = axes[row]
        ax_z.plot(t, z_log, linewidth=2)
        ax_z.axhline(0, color="gray", linestyle="--", linewidth=1)
        ax_z.axvline(2.0, color="red", linestyle=":", linewidth=1, label="gust")
        ax_z.set_ylabel("height error (cm)")
        ax_z.set_title(label, fontsize=10)
        ax_z.legend(loc="upper right", fontsize=7)

        ax_flap.plot(t, flap_log, linewidth=1.5, color="darkorange")
        ax_flap.axhline(0, color="gray", linestyle="--", linewidth=1)
        ax_flap.set_ylabel("flap deflection (deg)")

    axes[-1][0].set_xlabel("time (s)")
    axes[-1][1].set_xlabel("time (s)")
    fig.suptitle(
        f"Heave response, physics-based plant (trim AoA ~{alpha_trim_reported:.2f} deg at "
        "cruise) -- gains need retuning vs. v1, units are now real flap degrees, not generic",
        fontsize=10,
    )
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "flight_sim_v2_comparison.png"), dpi=130)
    print("Saved plot to flight_sim_v2_comparison.png")
    print(f"\nTrim AoA at cruise speed: {alpha_trim_reported:.2f} deg "
          "(this is the flap's steady baseline position -- the PID output above is added on top of it)")
    print("\nReminder: added_mass_kg and damping_ns_per_m are placeholders (see module")
    print("docstring) -- these gains are a demonstration of the method, not a number to")
    print("carry to the real boat. Retune here as your real numbers firm up, then again")
    print("on the actual bench test.")
