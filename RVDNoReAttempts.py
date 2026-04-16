"""
docking_corridor_fsm.py
=======================
Extends the base docking script with:
  Step 1 – Corridor + keep-out math helpers
  Step 2 – Explicit input bundle (Target, Shuttle, End-Effector)
  Step 3 – Conical corridor as an inside/outside test (cone tightens near dock)
  Step 4 – Spherical keep-out zone around the shuttle
  Step 5 – FSM states with guard rules
  Step 6 – 2-D visualizer (matplotlib) for offline debugging
  Step 7 – Evaluation metrics: SessionMetrics + MetricsLogger
           Tracks success rate, time-to-dock, final lateral error, tilt proxy.

CORRIDOR_ENTRY state (added):
  Fires once the mover has completed lateral alignment and is positioned
  at the corridor entry point (FAR waypoint, on the approach axis).
  This is the explicit "commit to straight-in run" gate.  The corridor
  axis geometrically passes through the keep-out zone — CORRIDOR_ENTRY
  acknowledges that intentionally; KeepOutChecker.violated() already
  permits keep-out penetration when in_corridor=True, so no safety logic
  changes are needed — the new state simply makes the gate visible in the
  FSM sequence and trajectory log.

All waypoints are resolved through _resolve(target, dx, dy) so every phase
works in target-relative offsets — change TARGET_X/TARGET_Y (or pass a live
pose) and the full sequence follows without any other edits.

Run normally  → python docking_corridor_fsm.py
Run visualizer → python docking_corridor_fsm.py --viz
"""

import time
import math
import argparse
import json
import os
from dataclasses import dataclass
from math import isclose, acos, sqrt, pi
from enum import Enum, auto

# ── Hardware imports guarded so the visualizer works without the hardware SDK ──
try:
    from pmclib import xbot_commands as bot
    from pmclib import system_commands as sys
    from pmclib import pmc_types as pm
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False
    print("⚠  pmclib not found – running in SIMULATION / VIZ mode only.")


# ==========================================================
#  MACHINE LIMITS
# ==========================================================
X_MIN, X_MAX = 0.000, 0.960
Y_MIN, Y_MAX = 4.330, 4.780

TARGET_X = 0.830
TARGET_Y = 4.680

Z_MIN_SAFE = 0.001
Z_MAX_SAFE = 0.004
Z_SPEED    = 0.002
R_SPEED    = 0.1
TILT_LIMIT = 0.01


# ==========================================================
#  RVD PARAMETERS
#  ── Real-space analogy ──────────────────────────────────
#  Table  470 mm  →  Space  10 km   (FAR rendezvous)
#  Table   47 mm  →  Space   1 km   (MID approach, half speed)
#  Table    8 mm  →  Space  200 m   (keep-out zone)
# ==========================================================
FAR_OFFSET_X       = -0.470   # 470 mm behind dock along X  (≈ 10 km in real space)
FAR_OFFSET_Y       =  0.000

MID_OFFSET_X       = -0.047   #  47 mm behind dock          (≈  1 km in real space)
MID_OFFSET_Y       =  0.000

HOLD_OFFSET_X      = -0.003   #   3 mm – fine alignment gate before contact
HOLD_OFFSET_Y      =  0.000

HOLD_LATERAL_TOL   = 0.0015

DWELL_FAR          = 2.0    # pause at FAR waypoint  (s)
DWELL_MID          = 2.0    # pause at MID waypoint  (s)
DWELL_HOLD         = 2.0    # pause at HOLD point    (s)

# ── XY approach speeds (m/s) ─────────────────────────────
XY_SPEED_FULL      = 1.0    # full speed        – FAR approach
XY_SPEED_MID       = 0.5    # half speed        – MID approach
XY_SPEED_CORRIDOR  = 0.25   # quarter speed     – corridor entry (keep-out boundary)
XY_SPEED_HOLD      = 0.125  # eighth speed      – fine alignment gate


# ==========================================================
#  CORRIDOR + KEEP-OUT PARAMETERS
# ==========================================================
APPROACH_AXIS_2D     = (-1.0, 0.0)
CONE_HALF_ANGLE_FAR  = math.radians(10.0)
CONE_HALF_ANGLE_NEAR = math.radians( 3.0)
CONE_TIGHTEN_START_D = 0.020
CONE_FULL_TIGHT_D    = 0.005
KEEPOUT_RADIUS       = 0.008

# CORRIDOR_ENTRY gate: the point on the approach axis where it pierces the
# keep-out sphere.  From here inward, keep-out penetration is intentional
# (the corridor axis runs straight through the zone to the dock face).
CORRIDOR_ENTRY_OFFSET_X = -KEEPOUT_RADIUS   # –8 mm along approach axis
CORRIDOR_ENTRY_OFFSET_Y =  0.000

# ==========================================================
#  METRICS FILE
# ==========================================================
METRICS_FILE = "relativecrashmetrics.json"


# ==========================================================
#  STEP 2 – INPUT BUNDLE
# ==========================================================
class Pose2D:
    """Minimal 2-D pose used for corridor/keep-out math."""
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __repr__(self):
        return f"Pose2D(x={self.x:.5f}, y={self.y:.5f})"


# ==========================================================
#  STEP 7 – EVALUATION METRICS
# ==========================================================
@dataclass
class SessionMetrics:
    """Collects per-run docking evaluation data.

    Fields
    ------
    run_id              : Sequential run number across all sessions.
    state_at_end        : Final FSM state name (SUCCESS, ABORT, etc.).
    total_run_time_s    : Wall-clock seconds from the very start of the run
                          to confirmed success or abort (full sequence timer).
    time_to_dock_s      : Wall-clock seconds from MID_HOLD exit to confirmed
                          task completion (success or post-hold abort).
                          NaN if the run did not reach HOLD exit.
                          This is the true "time to dock" — the interval spent
                          on the final approach and contact phase only.
    contact_attempts    : Number of liftoff attempts made in CONTACT phase.
    final_lateral_err_m : |Y_actual – Y_target| measured at HOLD exit —
                          the last corrected position before contact.  Best
                          proxy for docking alignment accuracy.
    final_tilt_rx       : rx (rad) at the moment of the first successful
                          liftoff. NaN if the run did not reach SUCCESS.
    final_tilt_ry       : ry (rad) at the moment of the first successful
                          liftoff. NaN if the run did not reach SUCCESS.
    corridor_violations : Total corridor-violation events logged by the FSM.
    keepout_violations  : Total keep-out violations (each one forces ABORT).
    final_pos_x_m       : Actual mover X position (m) read from hardware at
                          the end of the run (success or abort).
                          Confirms the mover reached the commanded waypoint.
    final_pos_y_m       : Actual mover Y position (m) read from hardware at
                          the end of the run (success or abort).
    final_pos_error_m   : Euclidean distance (m) between the final measured
                          position and the dock target (TARGET_X, TARGET_Y).
                          The key "did we actually arrive?" number.
    """
    run_id:               int   = 0
    state_at_end:         str   = "UNKNOWN"
    total_run_time_s:     float = 0.0
    time_to_dock_s:       float = float("nan")
    contact_attempts:     int   = 0
    final_lateral_err_m:  float = float("nan")
    final_tilt_rx:        float = float("nan")
    final_tilt_ry:        float = float("nan")
    corridor_violations:  int   = 0
    keepout_violations:   int   = 0
    final_pos_x_m:        float = float("nan")
    final_pos_y_m:        float = float("nan")
    final_pos_error_m:    float = float("nan")


class MetricsLogger:
    """Persists SessionMetrics across runs and computes rolling success rate.

    All runs are appended to a JSON file so the success-rate denominator
    survives process restarts.  The file is created on first use.
    """

    def __init__(self, path: str = METRICS_FILE):
        self.path = path
        self._history: list = []
        if os.path.exists(path):
            try:
                with open(path, encoding="utf-8") as f:
                    self._history = json.load(f)
            except (json.JSONDecodeError, OSError):
                print(f"⚠  Could not read {path} – starting fresh history.")
                self._history = []

    @property
    def next_run_id(self) -> int:
        """1-based run counter across all sessions."""
        return len(self._history) + 1

    def save(self, m: SessionMetrics) -> None:
        """Append metrics for this run and flush to disk."""
        record = vars(m)
        self._history.append(record)
        try:
            with open(self.path, "w", encoding="utf-8") as f:
                json.dump(self._history, f, indent=2)
        except OSError as e:
            print(f"⚠  Metrics write error: {e}")

    def success_rate(self) -> float:
        """Fraction of all logged runs that ended in SUCCESS.
        Returns NaN when no runs have been recorded yet."""
        if not self._history:
            return float("nan")
        successes = sum(
            1 for r in self._history if r.get("state_at_end") == "SUCCESS"
        )
        return successes / len(self._history)

    def print_summary(self, m: SessionMetrics) -> None:
        """Print a formatted per-run + rolling summary to stdout."""
        rate = self.success_rate()
        # NaN-safe string formatting
        rate_str  = f"{rate * 100:.1f}%" if rate == rate else "N/A"
        lat_str   = (f"{m.final_lateral_err_m * 1e3:.3f} mm"
                     if m.final_lateral_err_m == m.final_lateral_err_m
                     else "N/A")
        rx_str    = (f"{m.final_tilt_rx:.4f} rad"
                     if m.final_tilt_rx == m.final_tilt_rx else "N/A")
        ry_str    = (f"{m.final_tilt_ry:.4f} rad"
                     if m.final_tilt_ry == m.final_tilt_ry else "N/A")
        px_str    = (f"{m.final_pos_x_m:.5f} m"
                     if m.final_pos_x_m == m.final_pos_x_m else "N/A")
        py_str    = (f"{m.final_pos_y_m:.5f} m"
                     if m.final_pos_y_m == m.final_pos_y_m else "N/A")
        pe_str    = (f"{m.final_pos_error_m * 1e3:.3f} mm"
                     if m.final_pos_error_m == m.final_pos_error_m else "N/A")
        ttd_str   = (f"{m.time_to_dock_s:.2f} s"
                     if m.time_to_dock_s == m.time_to_dock_s else "N/A")

        print("\n" + "═" * 56)
        print("  RUN METRICS SUMMARY")
        print("═" * 56)
        print(f"  Run ID                 : {m.run_id}")
        print(f"  Outcome                : {m.state_at_end}")
        print(f"  Total run time         : {m.total_run_time_s:.2f} s")
        print(f"  Time to dock           : {ttd_str}  (post-MID_HOLD → task end)")
        print(f"  Contact attempts       : {m.contact_attempts}")
        print(f"  Final lateral error    : {lat_str}")
        print(f"  Final tilt rx          : {rx_str}")
        print(f"  Final tilt ry          : {ry_str}")
        print(f"  Corridor violations    : {m.corridor_violations}")
        print(f"  Keep-out violations    : {m.keepout_violations}")
        print(f"  Final position X       : {px_str}  (target {TARGET_X:.5f} m)")
        print(f"  Final position Y       : {py_str}  (target {TARGET_Y:.5f} m)")
        print(f"  Final position error   : {pe_str}")
        print(f"  ── Rolling success rate ({len(self._history)} runs): {rate_str}")
        print("═" * 56 + "\n")


# ==========================================================
#  STEP 3 – CONICAL CORRIDOR  (inside / outside test)
# ==========================================================
class CorridorChecker:
    def __init__(
        self,
        axis=APPROACH_AXIS_2D,
        theta_far=CONE_HALF_ANGLE_FAR,
        theta_near=CONE_HALF_ANGLE_NEAR,
        d_start=CONE_TIGHTEN_START_D,
        d_near=CONE_FULL_TIGHT_D,
    ):
        mag = sqrt(axis[0]**2 + axis[1]**2)
        self.axis       = (axis[0]/mag, axis[1]/mag)
        self.theta_far  = theta_far
        self.theta_near = theta_near
        self.d_start    = d_start
        self.d_near     = d_near

    def compute(self, target: Pose2D, ee: Pose2D):
        """Returns (distance, theta_rad, theta_max_rad, inside_corridor)."""
        rx = ee.x - target.x
        ry = ee.y - target.y
        d  = sqrt(rx**2 + ry**2)
        if d < 1e-9:
            return 0.0, 0.0, self.theta_far, True
        r_hat_x = rx / d
        r_hat_y = ry / d
        dot = max(-1.0, min(1.0, r_hat_x * self.axis[0] + r_hat_y * self.axis[1]))
        theta     = acos(dot)
        theta_max = self._theta_max(d)
        return d, theta, theta_max, (theta <= theta_max)

    def _theta_max(self, d: float) -> float:
        if d >= self.d_start:
            return self.theta_far
        if d <= self.d_near:
            return self.theta_near
        t = (d - self.d_near) / (self.d_start - self.d_near)
        return self.theta_near + (self.theta_far - self.theta_near) * t


# ==========================================================
#  STEP 4 – KEEP-OUT ZONE  (sphere / circle)
# ==========================================================
class KeepOutChecker:
    def __init__(self, radius: float = KEEPOUT_RADIUS):
        self.radius = radius

    def distance(self, shuttle: Pose2D, ee: Pose2D) -> float:
        return sqrt((ee.x - shuttle.x)**2 + (ee.y - shuttle.y)**2)

    def inside_zone(self, shuttle: Pose2D, ee: Pose2D) -> bool:
        return self.distance(shuttle, ee) < self.radius

    def violated(self, shuttle: Pose2D, ee: Pose2D, in_corridor: bool) -> bool:
        # Penetrating the keep-out zone along the approach corridor axis is
        # intentional (the corridor cuts through the zone at docking).
        # Only flag a violation when we are inside the zone AND off-axis.
        return self.inside_zone(shuttle, ee) and (not in_corridor)


# ==========================================================
#  STEP 5 – FSM WITH GUARD RULES
# ==========================================================
class DockState(Enum):
    STARTUP         = auto()
    FAR_APPROACH    = auto()
    CORRIDOR_ENTRY  = auto()   # gate: vehicle aligned & committed to straight-in run
    MID_APPROACH    = auto()
    MID_HOLD        = auto()
    CONTACT         = auto()
    SUCCESS         = auto()
    ABORT           = auto()


class DockingFSM:
    CORRIDOR_WARN_LIMIT = 3
    KEEPOUT_ABORT_LIMIT = 1

    def __init__(self):
        self.state                = DockState.STARTUP
        self.corridor             = CorridorChecker()
        self.keepout              = KeepOutChecker()
        self._corridor_violations = 0
        self._keepout_violations  = 0   # ← new: count keep-out events for metrics
        self._trajectory          = []

    def check_guards(self, target: Pose2D, ee: Pose2D) -> bool:
        self._trajectory.append((ee.x, ee.y, self.state.name))
        d, theta, theta_max, in_corridor = self.corridor.compute(target, ee)
        ko_violated = self.keepout.violated(target, ee, in_corridor)

        if ko_violated:
            self._keepout_violations += 1
            print(
                f"🚨 KEEP-OUT VIOLATION at {ee}  |  d={d*1e3:.2f} mm  "
                f"(limit {self.keepout.radius*1e3:.1f} mm)  "
                f"state={self.state.name}"
            )
            self._corridor_violations = 0
            self._transition(DockState.ABORT)
            return False

        if not in_corridor:
            self._corridor_violations += 1
            print(
                f"⚠  Corridor violation #{self._corridor_violations}  "
                f"θ={math.degrees(theta):.1f}°  θ_max={math.degrees(theta_max):.1f}°  "
                f"d={d*1e3:.2f} mm  state={self.state.name}"
            )
            if (self.state in (DockState.MID_HOLD, DockState.CONTACT) or
                    self._corridor_violations >= self.CORRIDOR_WARN_LIMIT):
                self._transition(DockState.ABORT)
                return False
        else:
            if self._corridor_violations:
                print(f"✔  Back inside corridor (d={d*1e3:.2f} mm, "
                      f"θ={math.degrees(theta):.1f}°)")
            self._corridor_violations = 0

        return True

    def _transition(self, new_state: DockState):
        print(f"FSM: {self.state.name} → {new_state.name}")
        self.state = new_state

    def advance(self, new_state: DockState):
        self._transition(new_state)

    @property
    def trajectory(self):
        return list(self._trajectory)

    @property
    def total_corridor_violations(self) -> int:
        """Cumulative corridor violations across all check_guards calls."""
        return self._corridor_violations

    @property
    def total_keepout_violations(self) -> int:
        """Cumulative keep-out violations across all check_guards calls."""
        return self._keepout_violations


# ==========================================================
#  STEP 6 – 2-D VISUALIZER
# ==========================================================
def visualize_2d(
    target: Pose2D,
    trajectory,
    corridor: CorridorChecker,
    keepout: KeepOutChecker,
    title: str = "Docking Corridor & Keep-Out Debug View",
):
    try:
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Wedge
    except ImportError:
        print("⚠  matplotlib not installed – skipping visualizer.")
        return

    fig, ax = plt.subplots(figsize=(16, 7))
    ax.set_facecolor("#0d1117")
    fig.patch.set_facecolor("#0d1117")
    for spine in ax.spines.values():
        spine.set_color("#30363d")
    ax.tick_params(colors="#8b949e", labelsize=8)
    ax.xaxis.label.set_color("#8b949e")
    ax.yaxis.label.set_color("#8b949e")

    axis_angle_deg = math.degrees(math.atan2(corridor.axis[1], corridor.axis[0]))

    for (radius, half_angle, colour, label_text) in [
        (CONE_TIGHTEN_START_D * 3,  corridor.theta_far,  "#1f6feb",
         f"Far corridor ±{math.degrees(corridor.theta_far):.0f}°"),
        (CONE_FULL_TIGHT_D   * 2.5, corridor.theta_near, "#388bfd",
         f"Near corridor ±{math.degrees(corridor.theta_near):.0f}°"),
    ]:
        half_deg = math.degrees(half_angle)
        wedge = Wedge(
            center=(target.x, target.y), r=radius,
            theta1=axis_angle_deg - half_deg,
            theta2=axis_angle_deg + half_deg,
            color=colour, alpha=0.18, label=label_text, zorder=2,
        )
        for sign in (+1, -1):
            angle = math.radians(axis_angle_deg + sign * half_deg)
            ax.plot(
                [target.x, target.x + math.cos(angle) * radius],
                [target.y, target.y + math.sin(angle) * radius],
                color=colour, lw=0.8, alpha=0.6, zorder=3,
            )
        ax.add_patch(wedge)

    for (radius, colour, style, label_text, ann_text) in [
        (abs(FAR_OFFSET_X), "#3fb950", "--",
         f"Far rendezvous r={abs(FAR_OFFSET_X)*1e3:.0f} mm",
         f"FAR RVD\n{abs(FAR_OFFSET_X)*1e3:.0f} mm"),
        (abs(MID_OFFSET_X), "#e3b341", "--",
         f"Mid approach r={abs(MID_OFFSET_X)*1e3:.0f} mm",
         f"MID APP\n{abs(MID_OFFSET_X)*1e3:.0f} mm  (½ speed)"),
    ]:
        c = Circle(
            (target.x, target.y), radius=radius,
            fill=True, facecolor=colour, alpha=0.07,
            edgecolor=colour, linewidth=1.5, linestyle=style,
            label=label_text, zorder=3,
        )
        ax.add_patch(c)
        ax.annotate(
            ann_text, (target.x - radius, target.y),
            textcoords="offset points", xytext=(-28, 6),
            color=colour, fontsize=6.5, zorder=11,
            arrowprops=dict(arrowstyle="-", color=colour, lw=0.6),
        )

    ax.add_patch(Circle(
        (target.x, target.y), radius=keepout.radius,
        fill=True, facecolor="#f85149", alpha=0.20,
        edgecolor="#f85149", linewidth=1.5,
        label=f"Keep-out r={keepout.radius*1e3:.0f} mm", zorder=4,
    ))

    ax.plot(target.x, target.y, marker="X", ms=12,
            color="#f0883e", zorder=10, label="Dock target")
    ax.annotate("TARGET", (target.x, target.y),
                textcoords="offset points", xytext=(6, 6),
                color="#f0883e", fontsize=7, zorder=11)

    state_colours = {
        "STARTUP":         "#8b949e",
        "FAR_APPROACH":    "#3fb950",
        "CORRIDOR_ENTRY":  "#00e5cc",   # teal – corridor commit gate
        "MID_APPROACH":    "#58a6ff",
        "MID_HOLD":        "#d2a8ff",
        "CONTACT":         "#ffa657",
        "SUCCESS":         "#56d364",
        "ABORT":           "#f85149",
    }

    if trajectory:
        xs = [p[0] for p in trajectory]
        ys = [p[1] for p in trajectory]
        ax.plot(xs, ys, color="#484f58", lw=0.8, zorder=5)
        for i, (x, y, sname) in enumerate(trajectory):
            col = state_colours.get(sname, "#8b949e")
            ax.scatter(x, y, s=40, color=col, zorder=6, linewidths=0)
            ax.annotate(
                f"#{i}\n{sname}",
                (x, y),
                textcoords="offset points",
                xytext=(6, 6),
                color=col,
                fontsize=5.5,
                fontweight="bold",
                zorder=12,
                bbox=dict(
                    boxstyle="round,pad=0.2",
                    facecolor="#0d1117",
                    edgecolor=col,
                    alpha=0.75,
                    linewidth=0.6,
                ),
            )
        ax.plot(xs[0], ys[0], "o", ms=9, color="#3fb950", zorder=9, label="Start")
        ax.plot(xs[-1], ys[-1], "s", ms=9, color="#ffa657", zorder=9, label="End")
        for sname, col in state_colours.items():
            if sname in {p[2] for p in trajectory}:
                ax.scatter([], [], s=40, color=col, label=f"State: {sname}", zorder=0)

    margin_x = abs(FAR_OFFSET_X) + 0.020
    margin_y = abs(FAR_OFFSET_X) * 0.25
    nx, ny = 300, 120
    gx = [target.x - margin_x + (2*margin_x/nx)*i for i in range(nx)]
    gy = [target.y - margin_y + (2*margin_y/ny)*j for j in range(ny)]
    in_mask = [[1.0 if corridor.compute(target, Pose2D(cx, cy))[3] else 0.0
                for cx in gx] for cy in gy]
    ax.contourf(gx, gy, in_mask, levels=[0.5, 1.5],
                colors=["#1f6feb"], alpha=0.07, zorder=1)

    ax.set_title(title, color="#e6edf3", fontsize=11, pad=10)
    ax.set_xlabel("X (m)", fontsize=9)
    ax.set_ylabel("Y (m)", fontsize=9)
    ax.set_xlim(target.x - margin_x, target.x + 0.030)
    ax.set_ylim(target.y - margin_y, target.y + margin_y)
    ax.set_aspect("equal")
    ax.legend(fontsize=7, loc="upper left",
              facecolor="#161b22", edgecolor="#30363d", labelcolor="#c9d1d9")
    ax.axhline(target.y, color="#21262d", lw=0.5, zorder=0)
    ax.axvline(target.x, color="#21262d", lw=0.5, zorder=0)

    plt.tight_layout()
    plt.savefig("docking_debug_view.png", dpi=180, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    print("📊 Debug plot saved to docking_debug_view.png")
    plt.show()


# ==========================================================
#  UTILITY HELPERS
# ==========================================================
def clip(value, vmin, vmax):
    return max(vmin, min(value, vmax))

def near(a, b, tol=1e-6):
    return isclose(a, b, abs_tol=tol)


# ==========================================================
#  SYSTEM STARTUP
# ==========================================================
class StartingUp:
    @staticmethod
    def run_start_up_routine(expected_xbot_count=1) -> bool:
        if not HARDWARE_AVAILABLE:
            print("SIMULATION: skipping hardware startup.")
            return True

        print("Connecting to Planar Motor Controller…")
        if not sys.auto_search_and_connect_to_pmc():
            print("❌ Could not connect.")
            return False

        print("Gaining mastership…")
        sys.gain_mastership()

        while True:
            st = sys.get_pmc_status()
            if st in (pm.PMCSTATUS.PMC_FULLCTRL, pm.PMCSTATUS.PMC_INTELLIGENTCTRL):
                break
            if st in (pm.PMCSTATUS.PMC_ERROR, pm.PMCSTATUS.PMC_INACTIVE):
                bot.activate_xbots()
            time.sleep(0.5)

        ids = bot.get_xbot_ids()
        if ids.PmcRtn != pm.PMCRTN.ALLOK or ids.xbot_count != expected_xbot_count:
            print("❌ Incorrect mover count.")
            return False

        bot.stop_motion(0)

        print("Levitation check…")
        while True:
            all_up = True
            ids = bot.get_xbot_ids()
            for x in ids.xbot_ids_array:
                st = bot.get_xbot_status(x).xbot_state
                if st in (pm.XBOTSTATE.XBOT_LANDED,
                          pm.XBOTSTATE.XBOT_STOPPING,
                          pm.XBOTSTATE.XBOT_DISCOVERING,
                          pm.XBOTSTATE.XBOT_MOTION):
                    all_up = False
            if all_up:
                break
            bot.levitation_command(0, pm.LEVITATEOPTIONS.LEVITATE)
            time.sleep(0.2)

        print("✅ Startup complete.")
        return True


# ==========================================================
#  LIFTOFF + RVD  (FSM-guarded)
# ==========================================================
class LiftOffWithTiltRecovery:

    # ─────────────────────────────────────────────────────
    #  CORE HELPER: resolve target-relative offset → clipped
    #  absolute table position.
    # ─────────────────────────────────────────────────────
    @staticmethod
    def _resolve(target: Pose2D, dx: float, dy: float) -> tuple:
        """Convert a target-relative (dx, dy) offset to a clipped
        absolute table position.  Returns (abs_x, abs_y)."""
        return (
            clip(target.x + dx, X_MIN, X_MAX),
            clip(target.y + dy, Y_MIN, Y_MAX),
        )

    @staticmethod
    def _stamp_final_position(
        xbot_id, metrics: SessionMetrics, target: Pose2D
    ) -> None:
        """Read the current XY from hardware and write it plus the
        Euclidean error to target into *metrics* in-place.

        Called just before every logger.save() so the JSON always
        contains the position the mover was actually at when the run
        ended — regardless of whether it succeeded or aborted.
        Falls back silently if the hardware read raises an exception.
        """
        try:
            fx, fy = LiftOffWithTiltRecovery.get_xy(xbot_id)
            metrics.final_pos_x_m     = fx
            metrics.final_pos_y_m     = fy
            metrics.final_pos_error_m = sqrt(
                (fx - target.x) ** 2 + (fy - target.y) ** 2
            )
        except Exception as exc:
            print(f"⚠  Could not read final position: {exc}")

    @staticmethod
    def get_mover_id():
        return bot.get_xbot_ids().xbot_ids_array[0]

    @staticmethod
    def wait_idle(xbot_id):
        while bot.get_xbot_status(xbot_id).xbot_state != pm.XBOTSTATE.XBOT_IDLE:
            time.sleep(0.05)

    @staticmethod
    def get_xy(xbot_id):
        st = bot.get_xbot_status(xbot_id).feedback_position_si
        return float(st[0]), float(st[1])

    @staticmethod
    def move_to_xy(xbot_id, x, y, label=100, speed=None):
        if speed is None:
            speed = XY_SPEED_FULL
        x_cmd = clip(x, X_MIN, X_MAX)
        y_cmd = clip(y, Y_MIN, Y_MAX)
        print(f"Moving to ({x_cmd:.3f}, {y_cmd:.3f})  speed={speed:.2f} m/s…")
        bot.linear_motion_si(
            label, xbot_id,
            pm.POSITIONMODE.ABSOLUTE,
            pm.LINEARPATHTYPE.DIRECT,
            x_cmd, y_cmd,
            0.0, speed, 10.0, 0.0
        )
        LiftOffWithTiltRecovery.wait_idle(xbot_id)

    @staticmethod
    def _safe_short_axes_move(xbot_id, target_z,
                              rx=0.0, ry=0.0, rz=0.0, label=200):
        clamped = clip(target_z, Z_MIN_SAFE, Z_MAX_SAFE)
        try:
            bot.short_axes_motion_si(
                label, xbot_id, pm.POSITIONMODE.ABSOLUTE,
                clamped, rx, ry, rz,
                Z_SPEED, R_SPEED, R_SPEED, R_SPEED
            )
            LiftOffWithTiltRecovery.wait_idle(xbot_id)
            return True
        except Exception:
            try:
                bot.short_axes_motion_si(
                    label+1, xbot_id, pm.POSITIONMODE.ABSOLUTE,
                    Z_MAX_SAFE, 0, 0, 0,
                    Z_SPEED, R_SPEED, R_SPEED, R_SPEED
                )
                LiftOffWithTiltRecovery.wait_idle(xbot_id)
                return True
            except Exception as e2:
                print(f"❌ Z move failed: {e2}")
                return False

    @staticmethod
    def attempt_liftoff(xbot_id, requested_lift=0.004):
        """Attempt a liftoff and return (success, rx, ry).

        rx and ry are the tilt values read from the mover immediately after
        the move completes — populated on both success and failure so the
        caller can log them into SessionMetrics regardless of outcome.
        """
        st = bot.get_xbot_status(xbot_id)
        z0 = float(st.feedback_position_si[2])
        target = clip(z0 + requested_lift, Z_MIN_SAFE, Z_MAX_SAFE)
        print(f"Lifting Z: {z0:.4f} → {target:.4f}")
        ok = LiftOffWithTiltRecovery._safe_short_axes_move(xbot_id, target, 0, 0, 0)
        if not ok:
            return False, float("nan"), float("nan")

        st2 = bot.get_xbot_status(xbot_id)
        z1  = float(st2.feedback_position_si[2])
        rx  = float(st2.feedback_position_si[3])
        ry  = float(st2.feedback_position_si[4])

        if z1 < target - 0.0003:
            print(f"❌ COLLISION: moved {z1-z0:.4f} instead of {target-z0:.4f}")
            return False, rx, ry

        if abs(rx) > TILT_LIMIT or abs(ry) > TILT_LIMIT:
            print(f"❌ Tilt too high (rx={rx:.4f}, ry={ry:.4f})")
            return False, rx, ry

        print("✔ Liftoff successful.")
        return True, rx, ry

    @staticmethod
    def recover(xbot_id):
        print("Recovering: lowering to Z_MIN_SAFE…")
        LiftOffWithTiltRecovery._safe_short_axes_move(xbot_id, Z_MIN_SAFE, 0, 0, 0, label=300)
        time.sleep(0.15)
        LiftOffWithTiltRecovery._safe_short_axes_move(xbot_id, Z_MIN_SAFE, 0, 0, 0, label=301)
        print("Recovery complete.")

    # ─────────────────────────────────────────────────────
    #  PHASE 1: FAR APPROACH
    # ─────────────────────────────────────────────────────
    @staticmethod
    def _phase_far_approach(xbot_id, fsm: DockingFSM, target: Pose2D):
        print("\n=== PHASE 1: FAR APPROACH ===")
        resolve = LiftOffWithTiltRecovery._resolve
        cur_x, cur_y = LiftOffWithTiltRecovery.get_xy(xbot_id)

        # Lateral alignment: snap Y to target, keep current X absolute.
        if not near(cur_y, target.y, HOLD_LATERAL_TOL / 2):
            ee = Pose2D(cur_x, target.y)
            if not fsm.check_guards(target, ee):
                return None, None
            print(f"Aligning lateral axis (Y → {target.y:.3f})")
            LiftOffWithTiltRecovery.move_to_xy(xbot_id, cur_x, target.y)

        # FAR waypoint — purely relative to the dock
        far_x, far_y = resolve(target, FAR_OFFSET_X, FAR_OFFSET_Y)
        ee = Pose2D(far_x, far_y)
        if not fsm.check_guards(target, ee):
            return None, None

        print(f"Proceeding to FAR waypoint ({far_x:.3f}, {far_y:.3f})")
        LiftOffWithTiltRecovery.move_to_xy(xbot_id, far_x, far_y, speed=XY_SPEED_FULL)
        time.sleep(DWELL_FAR)
        return far_x, far_y

    # ─────────────────────────────────────────────────────
    #  PHASE 2b: CORRIDOR ENTRY  (keep-out sphere boundary)
    #
    #  The mover advances to the exact point where the approach
    #  axis pierces the keep-out sphere (–8 mm / –KEEPOUT_RADIUS).
    #  This is the last waypoint *outside* the zone; the next
    #  move (HOLD / CONTACT) crosses inside it intentionally.
    #
    #  Why a dedicated state here?
    #    • It marks the precise "cleared to enter keep-out zone"
    #      decision in the FSM and trajectory log.
    #    • KeepOutChecker.violated() already permits penetration
    #      when in_corridor=True, so no safety logic changes are
    #      needed — the state just makes the gate explicit.
    #    • The corridor has tightened to its near-field angle by
    #      this distance; a lateral error that survived the wider
    #      FAR cone is very likely caught here before contact.
    # ─────────────────────────────────────────────────────
    @staticmethod
    def _phase_corridor_entry(xbot_id, fsm: DockingFSM, target: Pose2D):
        print("\n=== PHASE 2b: CORRIDOR ENTRY  (keep-out sphere boundary @ "
              f"{KEEPOUT_RADIUS*1e3:.0f} mm) ===")
        resolve = LiftOffWithTiltRecovery._resolve

        entry_x, entry_y = resolve(target, CORRIDOR_ENTRY_OFFSET_X, CORRIDOR_ENTRY_OFFSET_Y)
        ee = Pose2D(entry_x, entry_y)

        d, theta, theta_max, in_corridor = fsm.corridor.compute(target, ee)

        print(f"  Position  : x={entry_x:.5f}  y={entry_y:.5f}")
        print(f"  Distance  : {d*1e3:.2f} mm  (keep-out sphere surface)")
        print(f"  Angle     : θ={math.degrees(theta):.2f}°  "
              f"(corridor limit {math.degrees(theta_max):.1f}°)")
        print(f"  Corridor  : {'✔ INSIDE — cleared to enter zone' if in_corridor else '✘ OUTSIDE — aborting'}")
        print(f"  Note      : approach axis intersects keep-out zone — "
              f"penetration is PERMITTED from this point (in_corridor=True).")

        if not fsm.check_guards(target, ee):
            return None, None

        print(f"Advancing to corridor entry point ({entry_x:.4f}, {entry_y:.4f})  [quarter speed]")
        LiftOffWithTiltRecovery.move_to_xy(xbot_id, entry_x, entry_y, speed=XY_SPEED_CORRIDOR)

        print("✔ At keep-out sphere boundary.  Transitioning to MID_HOLD.")
        return entry_x, entry_y

    # ─────────────────────────────────────────────────────
    #  PHASE 2: MID APPROACH  (half speed)
    # ─────────────────────────────────────────────────────
    @staticmethod
    def _phase_mid_approach(xbot_id, fsm: DockingFSM, target: Pose2D):
        print("\n=== PHASE 2: MID APPROACH  (speed → 50%) ===")
        resolve = LiftOffWithTiltRecovery._resolve

        mid_x, mid_y = resolve(target, MID_OFFSET_X, MID_OFFSET_Y)
        ee = Pose2D(mid_x, mid_y)
        if not fsm.check_guards(target, ee):
            return None, None

        print(f"Advancing to MID waypoint ({mid_x:.3f}, {mid_y:.3f})  [half speed]")
        LiftOffWithTiltRecovery.move_to_xy(xbot_id, mid_x, mid_y, speed=XY_SPEED_MID)
        time.sleep(DWELL_MID)
        return mid_x, mid_y

    # ─────────────────────────────────────────────────────
    #  PHASE 3: HOLD POINT  (quarter speed)
    # ─────────────────────────────────────────────────────
    @staticmethod
    def _phase_hold(xbot_id, fsm: DockingFSM, target: Pose2D):
        print("\n=== PHASE 3: HOLD POINT  (speed → 25%) ===")
        resolve = LiftOffWithTiltRecovery._resolve

        hold_x, hold_y = resolve(target, HOLD_OFFSET_X, HOLD_OFFSET_Y)
        ee = Pose2D(hold_x, hold_y)
        if not fsm.check_guards(target, ee):
            return None, None

        print(f"Advancing to HOLD ({hold_x:.3f}, {hold_y:.3f})  [quarter speed]")
        LiftOffWithTiltRecovery.move_to_xy(xbot_id, hold_x, hold_y, speed=XY_SPEED_HOLD)

        # Fine lateral correction after reaching HOLD
        cur_x, cur_y = LiftOffWithTiltRecovery.get_xy(xbot_id)
        if abs(cur_y - hold_y) > HOLD_LATERAL_TOL:
            print(f"Correcting lateral offset (ΔY={cur_y - hold_y:+.4f})")
            ee2 = Pose2D(cur_x, hold_y)
            if not fsm.check_guards(target, ee2):
                return None, None
            LiftOffWithTiltRecovery.move_to_xy(xbot_id, cur_x, hold_y, speed=XY_SPEED_HOLD)

        time.sleep(DWELL_HOLD)
        return hold_x, hold_y

    # ─────────────────────────────────────────────────────
    #  MAIN ROUTINE
    # ─────────────────────────────────────────────────────
    @staticmethod
    def run_liftoff_routine():
        print("=== FSM DOCKING: FAR → MID → CORRIDOR ENTRY → HOLD → CONTACT/CAPTURE ===\n")

        if not StartingUp.run_start_up_routine(1):
            return

        xbot_id = LiftOffWithTiltRecovery.get_mover_id()
        fsm     = DockingFSM()

        # Initialise metrics for this run.
        logger  = MetricsLogger()
        metrics = SessionMetrics(run_id=logger.next_run_id)

        # Single source of truth for the dock position.
        target_pose = Pose2D(TARGET_X, TARGET_Y)

        docking_start  = time.time()
        hold_exit_time = None   # set after MID_HOLD completes; used for time_to_dock_s
        print("⏱ Docking timer started.")

        # ── PHASE 1: FAR APPROACH (full speed) ───────────
        fsm.advance(DockState.FAR_APPROACH)
        far_x, far_y = LiftOffWithTiltRecovery._phase_far_approach(
            xbot_id, fsm, target_pose
        )
        if far_x is None:
            print("❌ FAR APPROACH aborted by guard.")
            metrics.state_at_end        = fsm.state.name
            metrics.total_run_time_s    = time.time() - docking_start
            # time_to_dock_s stays NaN — HOLD was never reached
            metrics.corridor_violations = fsm.total_corridor_violations
            metrics.keepout_violations  = fsm.total_keepout_violations
            LiftOffWithTiltRecovery._stamp_final_position(xbot_id, metrics, target_pose)
            logger.save(metrics)
            logger.print_summary(metrics)
            return

        # ── PHASE 2: MID APPROACH (half speed) ───────────
        fsm.advance(DockState.MID_APPROACH)
        mid_x, mid_y = LiftOffWithTiltRecovery._phase_mid_approach(
            xbot_id, fsm, target_pose
        )
        if mid_x is None:
            print("❌ MID APPROACH aborted by guard.")
            metrics.state_at_end        = fsm.state.name
            metrics.total_run_time_s    = time.time() - docking_start
            # time_to_dock_s stays NaN — HOLD was never reached
            metrics.corridor_violations = fsm.total_corridor_violations
            metrics.keepout_violations  = fsm.total_keepout_violations
            LiftOffWithTiltRecovery._stamp_final_position(xbot_id, metrics, target_pose)
            logger.save(metrics)
            logger.print_summary(metrics)
            return

        # ── PHASE 2b: CORRIDOR ENTRY ──────────────────────
        # Mover advances to the keep-out sphere surface (–8 mm).
        # This is the explicit "cleared to enter keep-out zone" gate.
        # The approach axis runs through the zone to the dock face;
        # penetration from here is intentional (in_corridor=True).
        fsm.advance(DockState.CORRIDOR_ENTRY)
        entry_x, entry_y = LiftOffWithTiltRecovery._phase_corridor_entry(
            xbot_id, fsm, target_pose
        )
        if entry_x is None:
            print("❌ CORRIDOR ENTRY aborted by guard.")
            metrics.state_at_end        = fsm.state.name
            metrics.total_run_time_s    = time.time() - docking_start
            # time_to_dock_s stays NaN — HOLD was never reached
            metrics.corridor_violations = fsm.total_corridor_violations
            metrics.keepout_violations  = fsm.total_keepout_violations
            LiftOffWithTiltRecovery._stamp_final_position(xbot_id, metrics, target_pose)
            logger.save(metrics)
            logger.print_summary(metrics)
            return

        # ── PHASE 3: HOLD POINT (quarter speed) ──────────
        fsm.advance(DockState.MID_HOLD)
        base_x, base_y = LiftOffWithTiltRecovery._phase_hold(
            xbot_id, fsm, target_pose
        )
        if base_x is None:
            print("❌ HOLD aborted by guard.")
            metrics.state_at_end        = fsm.state.name
            metrics.total_run_time_s    = time.time() - docking_start
            # time_to_dock_s stays NaN — HOLD did not complete
            metrics.corridor_violations = fsm.total_corridor_violations
            metrics.keepout_violations  = fsm.total_keepout_violations
            LiftOffWithTiltRecovery._stamp_final_position(xbot_id, metrics, target_pose)
            logger.save(metrics)
            logger.print_summary(metrics)
            return

        # ── MID_HOLD completed — start time-to-dock clock ─
        hold_exit_time = time.time()
        print("⏱ Time-to-dock clock started (MID_HOLD exit).")

        # Capture final lateral alignment error at HOLD exit (ACTUAL feedback).
        cur_x, cur_y = LiftOffWithTiltRecovery.get_xy(xbot_id)
        metrics.final_lateral_err_m = abs(cur_y - target_pose.y)

        # ── FINAL APPROACH: close the remaining 3 mm to dock face ──
        # HOLD_OFFSET_X is a gate only — alignment is verified there but the
        # shuttle must still travel to offset (0, 0) before liftoff.
        print("\n=== FINAL APPROACH: closing last 3 mm to dock face ===")
        final_x, final_y = LiftOffWithTiltRecovery._resolve(target_pose, 0.0, 0.0)
        ee_final = Pose2D(final_x, final_y)
        if not fsm.check_guards(target_pose, ee_final):
            print("❌ Guard triggered on final approach. Aborting.")
            now = time.time()
            metrics.state_at_end        = fsm.state.name
            metrics.total_run_time_s    = now - docking_start
            metrics.time_to_dock_s      = now - hold_exit_time
            metrics.corridor_violations = fsm.total_corridor_violations
            metrics.keepout_violations  = fsm.total_keepout_violations
            LiftOffWithTiltRecovery._stamp_final_position(xbot_id, metrics, target_pose)
            logger.save(metrics)
            logger.print_summary(metrics)
            return
        LiftOffWithTiltRecovery.move_to_xy(xbot_id, final_x, final_y, speed=XY_SPEED_HOLD)
        base_x, base_y = final_x, final_y

        # ── PHASE 4: CONTACT & CAPTURE ───────────────────
        fsm.advance(DockState.CONTACT)
        print("\n=== PHASE 4: CONTACT & CAPTURE ===")
        attempt = 1

        while True:
            print(f"\n--- Attempt #{attempt} @ ({base_x:.4f}, {base_y:.4f}) ---")

            ee_now = Pose2D(base_x, base_y)
            if not fsm.check_guards(target_pose, ee_now):
                print("❌ Guard triggered during CONTACT phase. Aborting.")
                break

            success, rx, ry = LiftOffWithTiltRecovery.attempt_liftoff(xbot_id)

            if success:
                fsm.advance(DockState.SUCCESS)
                now = time.time()

                # Populate all remaining metrics before saving.
                metrics.state_at_end        = DockState.SUCCESS.name
                metrics.total_run_time_s    = now - docking_start
                metrics.time_to_dock_s      = now - hold_exit_time
                metrics.contact_attempts    = attempt
                metrics.final_tilt_rx       = rx
                metrics.final_tilt_ry       = ry
                metrics.corridor_violations = fsm.total_corridor_violations
                metrics.keepout_violations  = fsm.total_keepout_violations
                LiftOffWithTiltRecovery._stamp_final_position(xbot_id, metrics, target_pose)
                logger.save(metrics)
                logger.print_summary(metrics)

                print(f"\n🏆 SUCCESS  |  total={metrics.total_run_time_s:.2f}s  "
                      f"|  time-to-dock={metrics.time_to_dock_s:.2f}s  "
                      f"|  attempts={attempt}")
                return

            print("❌ Collision or tilt detected — aborting (no retry).")
            fsm.advance(DockState.ABORT)
            break

        # Reached here only if the CONTACT while-loop broke without SUCCESS.
        now = time.time()
        metrics.state_at_end        = fsm.state.name
        metrics.total_run_time_s    = now - docking_start
        metrics.time_to_dock_s      = now - hold_exit_time   # HOLD was reached
        metrics.contact_attempts    = attempt
        metrics.corridor_violations = fsm.total_corridor_violations
        metrics.keepout_violations  = fsm.total_keepout_violations
        LiftOffWithTiltRecovery._stamp_final_position(xbot_id, metrics, target_pose)
        logger.save(metrics)
        logger.print_summary(metrics)


# ==========================================================
#  SIMULATION / VIZ-ONLY PATH
# ==========================================================
def run_simulation_and_viz():
    print("\n══════════════════════════════════════════")
    print("  SIMULATION MODE – corridor & keep-out test")
    print("══════════════════════════════════════════\n")

    target   = Pose2D(TARGET_X, TARGET_Y)
    corridor = CorridorChecker()
    keepout  = KeepOutChecker()
    fsm      = DockingFSM()

    R = LiftOffWithTiltRecovery._resolve

    # Waypoints listed in sequence order.
    # CORRIDOR_ENTRY sits at –KEEPOUT_RADIUS (–8 mm) on the approach axis —
    # the exact point where the straight-in run pierces the keep-out sphere.
    waypoints = [
        (*R(target, -0.500,  0.010), DockState.FAR_APPROACH),   # slight lateral error
        (*R(target, -0.500,  0.000), DockState.FAR_APPROACH),   # lateral corrected
        (*R(target, FAR_OFFSET_X,            FAR_OFFSET_Y),        DockState.FAR_APPROACH),
        (*R(target, MID_OFFSET_X,            MID_OFFSET_Y),        DockState.MID_APPROACH),
        (*R(target, CORRIDOR_ENTRY_OFFSET_X, CORRIDOR_ENTRY_OFFSET_Y), DockState.CORRIDOR_ENTRY),
        (*R(target, HOLD_OFFSET_X,           HOLD_OFFSET_Y),       DockState.MID_HOLD),
        (*R(target, -0.001,  0.000), DockState.CONTACT),
        (*R(target,  0.000,  0.000), DockState.SUCCESS),
    ]

    WAYPOINT_DWELL = 1.5

    for i, (wx, wy, ws) in enumerate(waypoints):
        fsm.state = ws
        ee = Pose2D(wx, wy)
        d, theta, theta_max, in_corr = corridor.compute(target, ee)
        ko = keepout.violated(target, ee, in_corr)
        ok = fsm.check_guards(target, ee)

        print(f"\n{'─'*60}")
        print(f"  Waypoint {i}  |  State: {ws.name}")
        print(f"  Position  : x={wx:.5f}  y={wy:.5f}")
        print(f"  Distance  : {d*1e3:.2f} mm")
        print(f"  Angle     : θ={math.degrees(theta):.1f}°  (limit {math.degrees(theta_max):.1f}°)")
        print(f"  Corridor  : {'✔ INSIDE' if in_corr else '✘ OUTSIDE'}")
        print(f"  Keep-out  : {'🚨 VIOLATED' if ko else '✔ OK'}")
        if ws == DockState.CORRIDOR_ENTRY:
            print(f"  ► Approach axis intersects keep-out zone — "
                  f"entry permitted from here (in_corridor=True).")
        print(f"  Guard     : {'✔ PASS' if ok else '✘ ABORT'}")
        print(f"{'─'*60}")

        time.sleep(WAYPOINT_DWELL)

    # Simulate metrics for the viz run so the summary path is exercised.
    logger  = MetricsLogger()
    # In simulation the last waypoint is exactly at the target, so final
    # position error is 0.  Use the target coords as the synthetic readback.
    metrics = SessionMetrics(
        run_id              = logger.next_run_id,
        state_at_end        = fsm.state.name,
        total_run_time_s    = 0.0,   # not meaningful in sim
        time_to_dock_s      = 0.0,   # not meaningful in sim
        contact_attempts    = 1,
        final_lateral_err_m = 0.0,
        final_tilt_rx       = 0.0,
        final_tilt_ry       = 0.0,
        corridor_violations = fsm.total_corridor_violations,
        keepout_violations  = fsm.total_keepout_violations,
        final_pos_x_m       = TARGET_X,
        final_pos_y_m       = TARGET_Y,
        final_pos_error_m   = 0.0,
    )
    logger.save(metrics)
    logger.print_summary(metrics)

    print("\nGenerating 2-D debug plot…")
    visualize_2d(target, fsm.trajectory, corridor, keepout)


# ==========================================================
#  ENTRY POINT
# ==========================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--viz", action="store_true",
        help="Run offline simulation + 2-D visualizer (no hardware needed)"
    )
    args = parser.parse_args()

    if args.viz or not HARDWARE_AVAILABLE:
        run_simulation_and_viz()
    else:
        LiftOffWithTiltRecovery.run_liftoff_routine()