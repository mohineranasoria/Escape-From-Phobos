# Team Stella — V10 Autonomous GNC Technical Write-Up
**Competition:** SpaceTeams University #4 (STU4) — Fall 2026: *Escape from Phobos*  
**Team:** Team Stella  
**Controller:** `Controller.py` (V10 Ultra-Stable High-Score Lyapunov Multi-Vertex GNC)  
**Platform & Simulation:** SpaceTeamsPRO v0.55.0 (Win64) | `SimDynamX/STU4-Sim` commit `0d753a2` (`Backend Sim Version: 2026/11/11`, `seed = 8193`, unmodified)

---

## 1. Executive Summary
Team Stella's autonomous flight controller (`Controller.py`) solves the Phobos debris-field escape problem by combining four tightly coupled Guidance, Navigation, and Control (GNC) subsystems:

1. **Lyapunov Multi-Vertex Pure-Axis RCS Allocation** (`try_sync_vertex_table_from_sim`, `_select_vertex`) — unlocks 100%-L1-efficient 4-jet simplex vertices on `+Z`, `+X`, `-X`, `-Y`, `-Z` ($0.1929\text{ m/s}^2$, $2.01\times$ default `+Z` pure-axis thrust) and 3-jet vertices on `+Y` ($0.1447\text{ m/s}^2$), while using closed-loop Lyapunov angular-rate feedback to cancel residual torque to near zero and steer the gravity-compensated launch aim with zero extra propellant.
2. **Tumble-Immune 500-Rock Moving-Debris Tracker** (`update_tracks`) — tracks up to 500 radar contacts across SpaceCRAFT's $4.17\text{ s}$ round-robin radar sweep via gravity-compensated exponentially-weighted 7-sweep WLS fused with Doppler range-rate and timestamp-aligned shuttle state history (`deque(maxlen=600)`), with 2-sweep collision adaptation for bouncing small/medium debris ($R < 17.8\text{ m}$) and tumble immunity for giant boulders ($R \ge 17.8\text{ m}$).
3. **Single-Shot L1-Optimal Anti-Pinball Evasion & Two-Regime Monster Standoff** (`step`, Priority 1) — evaluates climb-aware closest-point-of-approach (CPA) threats up to $26.0\text{ s}$ ahead, triggers early fuel-optimal pure-axis avoidance burns with persistent `track_id` commitment, secondary-rock pinball lookahead, target-return tie-breaking, and a two-regime speed-proportional standoff for fast giant boulders.
4. **Ballistic CPA Zero-Effort-Miss (ZEM) & Zero-Chatter Hysteresis Coasting** (`step`, Priorities 2–3) — accelerates cleanly to $V_{\text{cruise}} = 9.15\text{ m/s}$, corrects lateral ballistic miss early without re-entering freshly dodged debris (per-threat opposite-axis lock), coasts ballistically for >100 s with hysteresis angular-rate damping ($0.012 / 0.004\text{ rad/s}$), and latches thruster cutoff cleanly inside the *AsteroidCycler* $50.0\text{ m}$ keep-out sphere.

Across two consecutive recorded live runs on the official `2026/11/11` simulation, Team Stella achieved scores of **53.6** and **55.1** (**54.35 two-run average**, only $1.5\text{ pts}$ spread), **174.42 s** average elapsed time, **48.37 kg** average fuel used, and **0.0 J** total damage (zero collisions).

---

## 2. GNC Architecture & Engineering Details

### 2.1 Lyapunov Multi-Vertex Pure-Axis RCS Allocator
The *SpaceBuggy* shuttle ($m = 3388.7\text{ kg}$, $I_{\text{sp}} = 80\text{ s}$, 38 RCS thrusters) is controlled through `JetSelectionActuator.py`, which maps `(CommandedAccel, CommandedAngAccel)` to jet firings via a simplex/LP solver. Standard zero-torque axis commands suffer from reduced net thrust (e.g., roughly half-capacity `+Z` thrust) due to thruster asymmetry around the center of mass.

- **Live Physics Sync (`try_sync_vertex_table_from_sim`):** As soon as `RCSConfig/Ready == True`, the controller reads all 38 thruster positions, thrust directions, and forces (`RCSConfig/Main0..37`) alongside `shuttle.getMass()`, `shuttle.getInertia(shuttleFrame)`, and thruster ceilings (`Thrusters/MaxAccel_m_s2 = 0.50`, `Thrusters/MaxAngAccel_rad_s2 = 0.05`). It computes exact 4-jet (`+Z`, `+X`, `-X`, `-Y`, `-Z`) and 3-jet (`+Y`) simplex command vertices whose net translation aligns $>99.5\%$ with the target body axis at 100% L1 propellant efficiency ($0.1929\text{ m/s}^2$ on five axes, $0.1447\text{ m/s}^2$ on `+Y`).
- **Closed-Loop Lyapunov Attitude Control (`_select_vertex`):** Because each multi-jet vertex generates a small known angular acceleration $\dot{\omega}_v$, switching across the axis vertex hull at every 60 Hz step minimizes the weighted Lyapunov cost $V(\omega + \dot{\omega}_v \Delta t - \omega_{\text{des}})$. During `CLIMB_Z`, `_compute_align_rate` smoothly drives body attitude toward the gravity-compensated launch intercept vector for **0.00 kg additional propellant**, keeping $\|\omega\| \approx 0.004\text{–}0.033\text{ rad/s}$ with zero attitude tumble.

### 2.2 Tumble-Immune 500-Rock Moving-Debris Tracker
SpaceCRAFT's radar queries all 500 debris objects (`RADAR_QUERY_COUNT = 500`, fetched every 4 ticks = 15 Hz) on a rotating ~4.17 s sweep with measurement noise and timestamp offsets.

- **Exact Timestamp Alignment:** A 600-sample ring buffer (`hist_t`, `hist_loc`, `hist_vel`) interpolates the shuttle's true Phobos-frame state at each radar return's exact measurement timestamp, eliminating self-motion velocity smear.
- **Gravity-Compensated WLS + Doppler Fusion:** Each tracked rock maintains a persistent `track_id` and estimates Phobos-frame position and velocity using exponentially-weighted 7-sweep WLS ($\tau = 14\text{ s}$) with Phobos spherical gravity compensation ($\mu_{\text{Phobos}} = 707500\text{ m}^3/\text{s}^2$) fused with radar radial range-rate.
- **2-Sweep Bounce Adaptation & Monster Tumble Immunity:** Non-destructive shadow hypotheses distinguish real rock-rock collisions from irregular mesh centroid jitter:
  - Small/medium debris ($R < 17.8\text{ m}$) rapidly adopt a new velocity hypothesis once confirmed across 2 consecutive sweeps.
  - Giant irregular boulders ($R \ge 17.8\text{ m}$) retain 7-sweep smoothing to prevent false velocity spikes caused by mesh tumbling.
  - Unconverged early tracks ($n = 1, 2, 3$) enforce strict distance/convergence gates so noisy distant returns never trigger wasteful phantom evades.

### 2.3 Single-Shot Anti-Pinball Evasion & Two-Regime Monster Standoff
At each 60 Hz step, `PhobosGNCController.step()` propagates every active track forward over a `TCA_HORIZON = 26.0 s` horizon (using climb-aware relative velocity lookahead during `CLIMB_Z`, `CLIMB_PRED_GAIN = 0.35`):

- **Adaptive Clearance Envelope:** Effective clearance radius combines shuttle bounding radius (`SHIP_BOUND_RADIUS = 3.2 m`), debris radius, proportional mesh margin (`BUF_MIN = 2.5 m`, `BUF_SCALE = 0.18`), and a two-regime speed-proportional standoff for giant boulders ($R \ge 17.8\text{ m}$): base ramp (`CHAOS_BASE_SCALE = 0.16`, max `4.8 m`) plus high-energy ramp above `2.60 m/s` (`CHAOS_HI_SCALE = 2.15`, max `16.5 m`, tapered down from `11.0 s` to `4.0 s` CPA).
- **Single-Shot Fuel-Optimal Execution:** When required avoidance acceleration exceeds `A_REQ_TRIG = 0.048 m/s²`, Priority 1 selects a single pure-axis high-thrust dodge direction (`0.193 m/s²`) evaluated against all secondary rocks (anti-pinball lookahead) and target-return alignment. Persistent `track_id` and axis hysteresis complete the dodge cleanly in one early burn instead of oscillatory late re-triggers.

### 2.4 Ballistic CPA ZEM & Zero-Chatter Hysteresis Coasting
- **Exact Ballistic Climb & Cruise:** Priority 2 climbs along `+Z` to `V_CRUISE = 9.15 m/s` (entering ballistic coast at `V_CRUISE - 0.06 m/s`).
- **Safe ZEM Correction:** Priority 3 computes the gravity-compensated closest-point-of-approach Zero-Effort-Miss relative to *AsteroidCycler* and executes pure-axis lateral corrections early—guarded by a `4.2 s` post-evade cooldown, converged-threat forward lookahead ($n \ge 3$ within $24\text{ s}$), and a per-threat opposite-axis lock so ZEM never steers back into a recently dodged rock.
- **Zero-Chatter Coast & Capture Cutoff:** Once aligned within the $50\text{ m}$ capture sphere, the shuttle coasts ballistically for over 100 seconds (only firing brief attitude-rate damping if $\|\omega\| > 0.012\text{ rad/s}$, exiting at $0.004\text{ rad/s}$, with an emergency tumble failsafe at $0.25\text{ rad/s}$). Upon entering `KEEP_OUT_CUTOFF = 49.2 m`, the controller latches `mission_won = True` and commands zero thrust.

---

## 3. Verified Consecutive Final Submission Runs
Both runs were executed back-to-back in a single continuous screen recording (`SpaceCRAFT 2026.11.21 - 07.46.11.05.mp4`) using the exact `Controller.py` in this repository (`MD5: 821E13B91ECDA895FA5E2EE7AD8EB80C`) on stock `SimDynamX/STU4-Sim` (`Backend Sim Version: 2026/11/11`, `seed = 8193`):

| Metric | Run 1 (`45f`) | Run 2 (`a01`) | Two-Run Average |
|---|:---:|:---:|:---:|
| **Final Score** | **53.6** | **55.1** | **54.35** |
| **Sim Elapsed Time (SET)** | 174.40 s | 174.44 s | **174.42 s** |
| **Total Fuel Used** | 48.94 kg | 47.79 kg | **48.37 kg** |
| **Total Damage Taken** | 0.0 J | 0.0 J | **0.0 J** |
| **Final Distance to Ship** | 49.72 m | 49.95 m | <= 50.0 m (Capture) |
| **Controller Errors / Warnings** | 0 | 0 | **0** |
| **Run UUID** | `45fe7e9f-40de-d624-aded-91910b230233` | `a011b6e1-45c6-926b-4002-52a38f155f8e` | Logged in `submission/logs/` |

**Flight Profile Highlights (from `COMPETITION_SIM_PySys_Controller.log`):**
- `t = 0.1–50.7 s`: `CLIMB_Z` with live 38-jet vertex sync (`mass=3388.70kg`), executing active `EVADE_+X` maneuvers at `t = 32.2 s` and `t = 43.2 s` while keeping tilt $\le 2.1^\circ$.
- `t = 50.7–68.8 s`: `ZEM_-X` alignment, one mid-flight `EVADE_+X` at `t = 56.2 s`, `COAST_HOLD` clear check, and final `ZEM_-X` refinement.
- `t = 68.8–174.4 s`: Continuous zero-propellant ballistic `COAST` ($\|\omega\| = 0.004\text{–}0.006\text{ rad/s}$) cleanly entering *AsteroidCycler*'s $50\text{ m}$ keep-out sphere with `0.0 J` damage.

---

## 4. Solution Validity & Competition Rules Compliance
Team Stella confirms full compliance with all [STU4 Rules & Info (#218)](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/218) and [Solution Validity](https://github.com/SimDynamX/STU4-Solution/wiki/3.-Competitor-Python-API#solution-validity) guidelines:

- **Only `Controller.py` modified:** `BasicController.py` and `UserAPI.py` are untouched from `SimDynamX/STU4-Solution`.
- **Zero simulation modifications:** `SimDynamX/STU4-Sim` (`commit 0d753a2`, `Backend Sim Version: 2026/11/11`) was used completely unmodified (`COMPETITION_SIM.simconf` MD5 `832D2F6FD19615D597DA835A05CC33F4`). No timescale, physics, entity, thruster, or scoring parameters were altered.
- **Permitted APIs only:** Debris sensing uses exclusively the official on-board radar (`shuttle.GetParamArray(..., ["Radar", "SortedDebrisData"])`). Vehicle actuation uses exclusively `SetCommand()` (`JetSelection/CommandedAccel` and `JetSelection/CommandedAngAccel`). No direct `debris.getLocation()` calls or direct state manipulation exist anywhere in the code.
- **Zero external dependencies:** Runs out of the box with SpaceTeamsPRO's built-in Python 3.12 + `spaceteams` + `numpy`.
