# Team Stella — V11 Autonomous GNC Technical Write-Up
**Competition:** SpaceTeams University #4 (STU4) — Fall 2026: *Escape from Phobos*  
**Team:** Team Stella  
**Controller:** `Controller.py` (V11 High-Score Zero-Fuel-Yield Lyapunov Multi-Vertex GNC)  
**Platform & Simulation:** SpaceTeamsPRO v0.55.0 (Win64) | `SimDynamX/STU4-Sim` commit `0d753a2` (`Backend Sim Version: 2026/11/11`, `FINAL_SUBMISSION.simconf`, `seed = 8193`, unmodified)

---

## 1. Executive Summary
Team Stella's autonomous flight controller (`Controller.py`) solves the Phobos debris-field escape problem by combining five tightly coupled Guidance, Navigation, and Control (GNC) subsystems:

1. **Lyapunov Multi-Vertex Pure-Axis RCS Allocation** (`try_sync_vertex_table_from_sim`, `_select_vertex`) — unlocks 100%-L1-efficient 4-jet simplex vertices on `+Z`, `+X`, `-X`, `-Y`, `-Z` ($0.1929\text{ m/s}^2$, $2.01\times$ default `+Z` pure-axis thrust) and 3-jet vertices on `+Y` ($0.1447\text{ m/s}^2$), while using closed-loop Lyapunov angular-rate feedback to cancel residual torque to near zero and steer the gravity-compensated launch aim with zero extra propellant.
2. **Zero-Fuel `CLIMB_YIELD` Timing Modulation** (`step`, climb threat gate) — when a crossing boulder intersects the projected `CLIMB_Z` corridor, V11 pauses `+Z` acceleration cleanly at $0.00\text{ kg/s}$ fuel and $0.0^\circ$ tilt (up to `MAX_YIELD_S = 10.5 s` with chatter-free hysteresis pad `YIELD_CLEAR_PAD = 3.5 m` and secondary neighbor validation) so the rock clears ahead without requiring an expensive lateral evasion burn.
3. **Tumble-Immune 500-Rock Moving-Debris Tracker** (`update_tracks`) — tracks up to 500 radar contacts across SpaceCRAFT's $4.17\text{ s}$ round-robin radar sweep via gravity-compensated exponentially-weighted 7-sweep WLS ($\tau = 14\text{ s}$, $\lambda_v = 5.0$) fused with Doppler range-rate and timestamp-aligned shuttle state history (`deque(maxlen=600)`), with 2-sweep collision adaptation for bouncing small/medium debris ($R < 17.8\text{ m}$) and tumble immunity + long-range convergence gating (`CHAOS_HI_N5_TCA = 21.5 s`) for giant boulders ($R \ge 17.8\text{ m}$).
4. **Single-Shot L1-Optimal Anti-Pinball Evasion & Two-Regime Monster Standoff** (`step`, Priority 1) — evaluates climb-aware closest-point-of-approach (CPA) threats up to $26.0\text{ s}$ ahead, triggers early fuel-optimal pure-axis avoidance burns with persistent `track_id` commitment, secondary-rock pinball lookahead, target-return tie-breaking, and a two-regime speed-proportional standoff (`CHAOS_HI_MAX = 16.5 m`) for fast giant boulders.
5. **Simpson Gravity, Online Coast Observer, Exact 3D CPA Edge ZEM & Zero-Waste Coasting** (`step`, Priorities 2–3) — accelerates cleanly to $V_{\text{cruise}} = 9.15\text{ m/s}$, propagates ballistic trajectories with Simpson midpoint Phobos gravity plus an online environmental acceleration observer (`a_env_extra`), targets the exact 3D CPA edge of the $50\text{ m}$ capture sphere (`44.0 / 42.5 / 41.0 m` stop band with `0.50 s` axis hysteresis and `23 s` debris lookahead), coasts ballistically for >100 s with widened zero-waste hysteresis damping (`0.024 / 0.008 rad/s`), and latches thruster cutoff cleanly inside `KEEP_OUT_CUTOFF = 49.2 m`.

Across two consecutive recorded live `FINAL_SUBMISSION.simconf` runs on the official `2026/11/11` simulation (`VIDEO.mp4`), Team Stella achieved scores of **54.3** and **60.6** (**57.45 two-run average**, up $+3.10\text{ pts}$ vs V10), **177.57 s** average elapsed time, **45.10 kg** average fuel used ($-3.27\text{ kg}$ vs V10), and **0.0 J** total damage (zero collisions). Across all 7 completed live V11 flights (6 `FINAL_SUBMISSION` + 1 `COMPETITION_SIM`), survival was **100% (7/7 at `0.0 J`)** with peaks of **62.9** (`COMPETITION_SIM`) and **61.8 / 60.6 / 60.5** (`FINAL_SUBMISSION`).

---

## 2. GNC Architecture & Engineering Details

### 2.1 Lyapunov Multi-Vertex Pure-Axis RCS Allocator
The *SpaceBuggy* shuttle ($m = 3388.7\text{ kg}$, $I_{\text{sp}} = 80\text{ s}$, 38 RCS thrusters) is controlled through `JetSelectionActuator.py`, which maps `(CommandedAccel, CommandedAngAccel)` to jet firings via a simplex/LP solver. Standard zero-torque axis commands suffer from reduced net thrust (e.g., roughly half-capacity `+Z` thrust) due to thruster asymmetry around the center of mass.

- **Live Physics Sync (`try_sync_vertex_table_from_sim`):** As soon as `RCSConfig/Ready == True`, the controller reads all 38 thruster positions, thrust directions, and forces (`RCSConfig/Main0..37`) alongside `shuttle.getMass()`, `shuttle.getInertia(shuttleFrame)`, and thruster ceilings (`Thrusters/MaxAccel_m_s2 = 0.50`, `Thrusters/MaxAngAccel_rad_s2 = 0.05`). It computes exact 4-jet (`+Z`, `+X`, `-X`, `-Y`, `-Z`) and 3-jet (`+Y`) simplex command vertices whose net translation aligns $>99.5\%$ with the target body axis at 100% L1 propellant efficiency ($0.1929\text{ m/s}^2$ on five axes, $0.1447\text{ m/s}^2$ on `+Y`).
- **Closed-Loop Lyapunov Attitude Control (`_select_vertex`):** Because each multi-jet vertex generates a small known angular acceleration $\dot{\omega}_v$, switching across the axis vertex hull at every 60 Hz step minimizes the weighted Lyapunov cost $V(\omega + \dot{\omega}_v \Delta t - \omega_{\text{des}})$. During `CLIMB_Z`, `_compute_align_rate` smoothly drives body attitude toward the gravity-compensated launch intercept vector for **0.00 kg additional propellant**, keeping $\|\omega\| \approx 0.005\text{–}0.015\text{ rad/s}$ and climb tilt $\le 1.1^\circ$ with zero attitude tumble.

### 2.2 Zero-Fuel `CLIMB_YIELD` Timing Modulation
During ascent (`CLIMB_Z`), lateral evasive maneuvers are doubly costly: they burn lateral propellant and introduce cross-track velocity that must later be cancelled by ZEM. V11 checks whether a converged crossing rock intersecting the projected climb trajectory clears cleanly if `+Z` thrust pauses momentarily:

- **Chatter-Free Yield Hysteresis:** Entering `CLIMB_YIELD` commands zero thrust ($0.00\text{ kg/s}$, $0.0^\circ$ tilt) and holds until the predicted full-climb miss margin reaches `r_safe + YIELD_CLEAR_PAD` (`+3.5 m`), validated against all neighboring rocks and bounded by `MAX_YIELD_S = 10.5 s`.
- **Measured Impact:** In Run 1 (`61e`), V11 executed `5.7 s` of zero-fuel `CLIMB_YIELD` between `t = 18.8–33.0 s`, clearing early debris without a single lateral climb evade and reaching cruise at `42.6 kg` fuel and `zem = 1.7 m`.

### 2.3 Tumble-Immune 500-Rock Moving-Debris Tracker
SpaceCRAFT's radar queries all 500 debris objects (`RADAR_QUERY_COUNT = 500`, fetched every 4 ticks = 15 Hz) on a rotating ~4.17 s sweep with measurement noise and timestamp offsets.

- **Exact Timestamp Alignment:** A 600-sample ring buffer (`hist_t`, `hist_loc`, `hist_vel`) interpolates the shuttle's true Phobos-frame state at each radar return's exact measurement timestamp, eliminating self-motion velocity smear.
- **Gravity-Compensated WLS + Doppler Fusion:** Each tracked rock maintains a persistent `track_id` and estimates Phobos-frame position and velocity using exponentially-weighted 7-sweep WLS ($\tau = 14\text{ s}$, $\lambda_v = 5.0$) with Phobos spherical gravity compensation ($\mu_{\text{Phobos}} = 707500\text{ m}^3/\text{s}^2$) fused with radar radial range-rate.
- **2-Sweep Bounce Adaptation & Monster Tumble Immunity:** Non-destructive shadow hypotheses distinguish real rock-rock collisions from irregular mesh centroid jitter:
  - Small/medium debris ($R < 17.8\text{ m}$) rapidly adopt a new velocity hypothesis once confirmed across 2 consecutive sweeps.
  - Giant irregular boulders ($R \ge 17.8\text{ m}$) retain 7-sweep smoothing to prevent false velocity spikes caused by mesh tumbling.
  - Unconverged early tracks ($n = 1, 2, 3$) enforce strict distance/convergence gates, and long-range high-energy monster standoffs beyond `CHAOS_HI_N5_TCA = 21.5 s` require $n \ge 5$ sweeps to prevent premature false evades.

### 2.4 Single-Shot Anti-Pinball Evasion & Two-Regime Monster Standoff
At each 60 Hz step, `PhobosGNCController.step()` propagates every active track forward over a `TCA_HORIZON = 26.0 s` horizon (using climb-aware relative velocity lookahead during `CLIMB_Z`, `CLIMB_PRED_GAIN = 0.35`):

- **Adaptive Clearance Envelope:** Effective clearance radius combines shuttle bounding radius (`SHIP_BOUND_RADIUS = 3.2 m`), debris radius, proportional mesh margin (`BUF_MIN = 2.5 m`, `BUF_SCALE = 0.18`), and a two-regime speed-proportional standoff for giant boulders ($R \ge 17.8\text{ m}$): base ramp (`CHAOS_BASE_SCALE = 0.16`, max `4.8 m`) plus high-energy ramp above `2.60 m/s` (`CHAOS_HI_SCALE = 2.15`, max `16.5 m`, tapered down from `11.0 s` to `4.0 s` CPA).
- **Single-Shot Fuel-Optimal Execution:** When required avoidance acceleration exceeds `A_REQ_TRIG = 0.048 m/s²`, Priority 1 selects a single pure-axis high-thrust dodge direction (`0.193 m/s²`) evaluated against all secondary rocks (anti-pinball lookahead) and target-return alignment. Persistent `track_id` and axis hysteresis complete the dodge cleanly in one early burn instead of oscillatory late re-triggers.

### 2.5 Simpson Gravity, Online Coast Observer, Exact 3D CPA Edge ZEM & Zero-Waste Coasting
- **Exact Ballistic Climb & Cruise:** Priority 2 climbs along `+Z` to `V_CRUISE = 9.15 m/s` (entering ballistic coast at `V_CRUISE - 0.06 m/s`).
- **Simpson Midpoint Gravity + Online Environmental Observer (`a_env_extra`):** During ballistic propagation, the controller integrates Phobos gravity via Simpson midpoint quadrature and continuously estimates residual unmodeled accelerations (`a_env_extra`) during unpowered coast, keeping predicted 3D CPA miss accurate to sub-meter levels across 100+ s coasts.
- **Exact 3D CPA Edge ZEM (`44.0 / 42.5 / 41.0 m`):** Rather than wasting fuel aiming at the dead center (`0 m`) of *AsteroidCycler*, Priority 3 evaluates the true 3D CPA distance (`norm(zem_vec)`) and terminates ZEM corrections once safely inside the edge stop band (`44.0 / 42.5 / 41.0 m`), guarded by `0.50 s` axis hysteresis, a post-evade cooldown, `23 s` directional debris lookahead, and a per-threat opposite-axis lock.
- **Zero-Waste Hysteresis Coast & Capture Cutoff:** Once aligned within the $50\text{ m}$ capture sphere, the shuttle coasts ballistically for over 100 seconds (widened hysteresis attitude-rate damping `COAST_DAMP_ENTER = 0.024 rad/s`, `EXIT = 0.008 rad/s`, eliminating wasteful mid-coast damp pulses, with an emergency tumble failsafe at $0.25\text{ rad/s}$). Upon entering `KEEP_OUT_CUTOFF = 49.2 m`, the controller latches `mission_won = True` and commands zero thrust.

---

## 3. Verified Consecutive Final Submission Runs
Both runs were executed back-to-back in a single continuous screen recording (`VIDEO.mp4`, `6m 29.9s`, `732.32 MB`) using the exact `Controller.py` in this repository (`MD5: 09008B3222FD2389E459E0000CE31E1A`, `SHA256: CF0D11EF2678348C640FDB0E7A8B9C992141FEEFA865DF93B1ED63BA680AF521`) on stock `SimDynamX/STU4-Sim` (`FINAL_SUBMISSION.simconf`, `Backend Sim Version: 2026/11/11`, `seed = 8193`):

| Metric | Run 1 (`61e`) | Run 2 (`d04`) | Two-Run Average |
|---|:---:|:---:|:---:|
| **Final Score** | **54.3** | **60.6** | **57.45** |
| **Sim Elapsed Time (SET)** | 179.73 s | 175.40 s | **177.57 s** |
| **Total Fuel Used** | 46.84 kg | 43.35 kg | **45.10 kg** |
| **Total Damage Taken** | 0.0 J | 0.0 J | **0.0 J** |
| **Final Distance to Ship** | 49.53 m | 49.98 m | <= 50.0 m (Capture) |
| **Controller Errors / Warnings** | 0 | 0 | **0** |
| **Run UUID** | `61ea859f-4098-687b-1bb3-c4a6ba9bfc05` | `d04311e8-4114-7772-d6d9-5eb0374ae05f` | Logged in `submission/logs/` |

**Flight Profile Highlights (from `FINAL_SUBMISSION_PySys_Controller.log`):**
- **Run 1 (`61e` — Score `54.3`, `46.84 kg`, `179.73 s`, `0.0 J`):**
  - `t = 0.0–59.0 s`: `CLIMB_Z` with live 38-jet vertex sync (`mass=3388.70kg`), demonstrating `5.7 s` of zero-fuel `CLIMB_YIELD` (`t = 18.8–33.0 s`) to clear crossing climb debris at `0.00 kg/s` fuel and tilt $\le 1.0^\circ$, entering cruise at `42.6 kg` and `zem = 1.7 m`.
  - `t = 67.2–78.5 s`: Single-shot `EVADE_-X` (`42.6 -> 45.4 kg`), `COAST_HOLD`, and short exact-edge `ZEM_+X` (`45.4 -> 46.8 kg`, settling `zem = 40.2 m`).
  - `t = 78.5–179.7 s`: **101.2 s** of uninterrupted zero-fuel ballistic `COAST` ($\|\omega\| = 0.005\text{ rad/s}$), cleanly entering *AsteroidCycler*'s $50\text{ m}$ keep-out sphere at `49.53 m` with `0.0 J` damage.
- **Run 2 (`d04` — Score `60.6`, `43.35 kg`, `175.40 s`, `0.0 J`):**
  - `t = 0.0–52.9 s`: Uninterrupted `CLIMB_Z` (`0.0 s` yield, tilt $\le 1.1^\circ$, $\|\omega\| = 0.005\text{–}0.007\text{ rad/s}$), entering cruise at `42.5 kg` and `zem = 16.6 m`.
  - `t = 65.8–67.2 s`: Single `0.9 kg` `EVADE_+Y` burn (`42.5 -> 43.4 kg`), leaving `zem = 33.5 m` already inside the capture envelope — **zero ZEM burn needed**.
  - `t = 68.8–175.4 s`: **106.6 s** of uninterrupted zero-fuel ballistic `COAST` ($\|\omega\| = 0.006\text{ rad/s}$), cleanly entering *AsteroidCycler*'s $50\text{ m}$ keep-out sphere at `49.98 m` with `0.0 J` damage and **Score 60.6**.

**Offline 6-DOF Verification Suite (`0.274 ms/tick`):** **71 / 71 PASS (100.0%, `0.0 J` damage)** across live-reconstructed Phobos & Phobos+Mars/Coriolis worlds (10/10), core flagship worlds (10/10, `WorldA = 65.2`), multi-seed tumble/bounce stress tests (20/20), adversarial worlds (6/6, Mean `54.8`), and random moving worlds (25/25).

---

## 4. Solution Validity & Competition Rules Compliance
Team Stella confirms full compliance with all [STU4 Rules & Info (#218)](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/218) and [Solution Validity](https://github.com/SimDynamX/STU4-Solution/wiki/3.-Competitor-Python-API#solution-validity) guidelines:

- **Only `Controller.py` modified:** `BasicController.py` and `UserAPI.py` are untouched from `SimDynamX/STU4-Solution`.
- **Zero simulation modifications:** `SimDynamX/STU4-Sim` (`commit 0d753a2`, `Backend Sim Version: 2026/11/11`) was used completely unmodified (`FINAL_SUBMISSION.simconf` MD5 `B109312681B5518D57B36B9FD25DC120`). No timescale, physics, entity, thruster, or scoring parameters were altered.
- **Permitted APIs only:** Debris sensing uses exclusively the official on-board radar (`shuttle.GetParamArray(..., ["Radar", "SortedDebrisData"])`). Vehicle actuation uses exclusively `SetCommand()` (`JetSelection/CommandedAccel` and `JetSelection/CommandedAngAccel`). No direct `debris.getLocation()` calls or direct state manipulation exist anywhere in the code.
- **Zero external dependencies:** Runs out of the box with SpaceTeamsPRO's built-in Python 3.12 + `spaceteams` + `numpy`.
