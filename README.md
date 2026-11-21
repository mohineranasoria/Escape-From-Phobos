# 🚀 Escape from Phobos — Team Stella (STU4)

Official repository for **Team Stella** participating in the **SpaceTeams University #4 (STU4) — "Escape from Phobos"** competition on SpaceTeamsPRO v0.55.0.

---

## 🛰️ Mission & Challenge Overview
Following an asteroid impact on a mining base on Phobos, our autonomous controller pilots an evacuation shuttle (*SpaceBuggy*, mass $3388.7\text{ kg}$, 38 RCS jets) through a dense, dynamically moving 3D debris field in Phobos' low-gravity environment to rendezvous with the orbiting *AsteroidCycler* keep-out sphere ($\le 50\text{ m}$).

The objective is to maximize the competition score—optimizing both **flight time** and **propellant efficiency**—while achieving **zero collisions** ($0.0\text{ J}$ damage) consistently across consecutive runs.

---

## 🏆 Final Submission Performance (V10 — Two Consecutive Runs)

Two consecutive live `COMPETITION_SIM` runs were recorded back-to-back using this exact `Controller.py` (`MD5: 821E13B91ECDA895FA5E2EE7AD8EB80C`, `SHA256: 50AF0954E6375A3B1AB4DC081625181EF836AB3BD1A31A9D50F01DD50930D211`):

| Metric | Run 1 (`45f`) | Run 2 (`a01`) | **2-Run Average / Combined** |
|---|:---:|:---:|:---:|
| **Final Score** | **53.6** | **55.1** | **54.35** *(spread: 1.5 pts)* |
| **Sim Elapsed Time (SET)** | $174.40\text{ s}$ | $174.44\text{ s}$ | **$174.42\text{ s}$** |
| **Total Fuel Used** | $48.94\text{ kg}$ | $47.79\text{ kg}$ | **$48.37\text{ kg}$** |
| **Total Damage Taken** | $0.0\text{ J}$ | $0.0\text{ J}$ | **$0.0\text{ J}$ (Zero Collisions)** |
| **Final Distance to Ship** | $49.72\text{ m}$ | $49.95\text{ m}$ | **Within Keep-Out ($\le 50.0\text{ m}$)** |
| **Run UUID** | `45fe7e9f-40de-d624-aded-91910b230233` | `a011b6e1-45c6-926b-4002-52a38f155f8e` | Verified |

---

## 🧠 V10 Guidance, Navigation & Control Architecture

1. **Lyapunov Multi-Vertex Pure-Axis RCS Allocator (100% L1 Efficiency):**
   - Computes exact zero-torque 4-jet simplex vertices on `+Z`, `+X`, `-X`, `-Y`, `-Z` ($0.1929\text{ m/s}^2$, $2.01\times$ default `+Z` thrust) and 3-jet vertices on `+Y` ($0.1447\text{ m/s}^2$), automatically synced live from `RCSConfig`, `getMass()`, and `getInertia()`.
   - Uses closed-loop Lyapunov feedback on measured body angular rates (`ang_vel_body`) to cancel residual torque to near zero ($\|\omega\| \approx 0.004\text{–}0.021\text{ rad/s}$) and steer the gravity-compensated launch aim at $V_{\text{cruise}} = 9.15\text{ m/s}$ with zero extra propellant.

2. **Tumble-Immune 500-Rock Moving-Debris Tracker & 2-Sweep Bounce Adaptation:**
   - Tracks all 500 radar contacts across SpaceCRAFT's $4.17\text{ s}$ round-robin radar sweep using gravity-compensated exponentially-weighted 7-sweep WLS ($\tau = 14\text{ s}$) fused with Doppler range-rate measurements and exact timestamp-aligned shuttle state history.
   - Adapts rapidly to confirmed 2-sweep rock-rock collisions on small/medium debris ($R < 17.8\text{ m}$) while preserving 7-sweep smoothing on giant tumbling boulders ($R \ge 17.8\text{ m}$) to eliminate false-positive velocity spikes.
   - Enforces strict track-convergence gates on unconverged early tracks ($n = 1, 2, 3$) to prevent phantom distant evades.

3. **Single-Shot Anti-Pinball Evasion & Two-Regime Monster Standoff:**
   - Completes active avoidance burns cleanly in one fuel-optimal maneuver at high $t_{\text{cpa}}$ with persistent track-ID commitment hysteresis, active-threat priority tie-breaking, pinball secondary-rock lookahead, and a target-return dodge tie-breaker.
   - Applies lobe-safe close-range envelopes and a two-regime speed-proportional standoff for fast chaotic giant boulders ($R \ge 17.8\text{ m}$).

4. **Exact Ballistic CPA Zero-Effort-Miss (ZEM) & Zero-Chatter Coasting:**
   - Computes exact gravity-compensated closest-point-of-approach ZEM at the *AsteroidCycler*, cancels lateral drift early with converged-threat safety lookahead and a per-threat opposite-axis lock (never burning back into a freshly dodged forward rock), and transitions to zero-chatter hysteresis coast damping ($0.175\text{ ms/tick}$ CPU latency at 60 Hz) until thruster cutoff inside the $50\text{ m}$ capture sphere.

---

## ✅ 6-DOF Verification Summary
- **10/10 Flagship Production Scenarios** (reproduced live bounces, tumbling monster boulders, corridors, moving/static fields): **100% PASS** (Mean Score `54.2`, Std `5.5`, Min `47.9`)
- **20/20 Multi-Seed Stress Tests** (5x each across bounce, tumbling boulder, and irregular-mesh worlds): **100% PASS**
- **6/6 Adversarial Worlds** (+4.5 m boulder tumble): **100% PASS** (Mean Score `55.5`, Std `5.5`)
- **25/25 Random Moving Worlds** (+4.5 m boulder tumble): **100% PASS**

---

## 📁 Repository Structure
```
Escape-From-Phobos/
├── Controller.py          # V10 Ultra-Stable High-Score GNC Controller (Team Stella)
├── BasicController.py     # Baseline reference controller
├── UserAPI.py             # SpaceTeams API reference utilities and examples
├── README.md              # Project documentation and verified flight benchmarks
├── .gitattributes         # Git LFS & line ending attributes
└── .gitignore             # Standard Python / simulation gitignore
```

---

## 🛠️ Instructions to Run
1. Launch **SpaceCRAFT.exe** (SpaceTeamsPRO v0.55.0).
2. Open `COMPETITION_SIM.simconf` from `Repos/STU4-Sim/Sims/`.
3. Ensure this `Controller.py` is loaded in your active solution repo (`Repos/STU4-Solution/Controller.py`).
4. Press Play to run the simulation.
