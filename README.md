| [Rules & Info (#218)](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/218) | [Leaderboard (#247)](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/247) | [Starter Code Repo](https://github.com/SimDynamX/STU4-Solution) | [STU4 Wiki](https://github.com/SimDynamX/STU4-Solution/wiki) | [Forum](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/categories/2026-escape-from-phobos) |
| --- | --- | --- | --- | --- |

# 🚀 Escape from Phobos — Team Stella (STU4 Final Submission)

Official solution repository for **Team Stella** competing in **SpaceTeams University #4 (STU4) — Fall 2026: "Escape from Phobos"** on SpaceTeamsPRO v0.55.0 (`SimDynamX/STU4-Sim` commit `0d753a2`, `Backend Sim Version: 2026/11/11`).

---

## 🛰️ Mission & Challenge Overview
Following an asteroid impact on a mining base on Phobos, our autonomous controller pilots an evacuation shuttle (*SpaceBuggy*, mass $3388.7\text{ kg}$, 38 RCS jets) through a dense, dynamically moving 500-rock 3D debris field in Phobos' low-gravity environment to rendezvous with the orbiting *AsteroidCycler* keep-out sphere ($\le 50\text{ m}$).

The objective is to maximize the competition score—optimizing both **flight time** and **propellant efficiency**—while achieving **zero collisions** ($0.0\text{ J}$ damage) consistently across two consecutive runs.

---

## 🏆 Final Submission Performance (V10 — Two Consecutive Runs)

Two consecutive live `COMPETITION_SIM` runs (`Backend Sim Version: 2026/11/11`, `seed = 8193`) were recorded back-to-back in a single continuous video using the exact `Controller.py` in this repository (`MD5: 821E13B91ECDA895FA5E2EE7AD8EB80C`, `SHA256: 50AF0954E6375A3B1AB4DC081625181EF836AB3BD1A31A9D50F01DD50930D211`):

| Metric | Run 1 (`45f`) | Run 2 (`a01`) | **2-Run Average / Combined** |
|---|:---:|:---:|:---:|
| **Final Score** | **53.6** | **55.1** | **54.35** *(spread: 1.5 pts)* |
| **Sim Elapsed Time (SET)** | $174.40\text{ s}$ | $174.44\text{ s}$ | **$174.42\text{ s}$** |
| **Total Fuel Used** | $48.94\text{ kg}$ | $47.79\text{ kg}$ | **$48.37\text{ kg}$** |
| **Total Damage Taken** | $0.0\text{ J}$ | $0.0\text{ J}$ | **$0.0\text{ J}$ (Zero Collisions)** |
| **Final Distance to Ship** | $49.72\text{ m}$ | $49.95\text{ m}$ | **Within Keep-Out ($\le 50.0\text{ m}$)** |
| **Run UUID** | `45fe7e9f-40de-d624-aded-91910b230233` | `a011b6e1-45c6-926b-4002-52a38f155f8e` | Verified (`submission/logs/`) |

Full unmodified STPro simulation logs for both runs are included under [`submission/logs/`](submission/logs/), and the complete technical report is in [`docs/SOLUTION_WRITEUP.md`](docs/SOLUTION_WRITEUP.md).

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

See [`docs/SOLUTION_WRITEUP.md`](docs/SOLUTION_WRITEUP.md) for the complete algorithmic specification and flight-log telemetry breakdown.

---

## ✅ Solution Validity & Verification Summary

- **No Sim Modifications:** 100% unmodified `SimDynamX/STU4-Sim` (`commit 0d753a2`, `Backend Sim Version: 2026/11/11`, `COMPETITION_SIM.simconf` MD5 `832D2F6FD19615D597DA835A05CC33F4`).
- **Official APIs Only:** Uses only the on-board radar (`Radar/SortedDebrisData`), shuttle/target navigation state, and thruster commands via `SetCommand()`.
- **Zero External Pip Dependencies:** Runs out of the box using only SpaceTeamsPRO's built-in Python 3.12 stdlib, `spaceteams`, and `numpy` (see [`requirements.txt`](requirements.txt)).
- **6-DOF Offline Stress-Test Suite:**
  - **10/10 Flagship Production Scenarios** (live bounces, tumbling monster boulders, corridors, moving/static fields): **100% PASS** (Mean Score `54.2`, Std `5.5`, Min `47.9`)
  - **20/20 Multi-Seed Stress Tests** (bounce, tumbling boulder, and irregular-mesh worlds): **100% PASS**
  - **6/6 Adversarial Worlds** (+4.5 m boulder tumble): **100% PASS** (Mean Score `55.5`, Std `5.5`)
  - **25/25 Random Moving Worlds** (+4.5 m boulder tumble): **100% PASS**

---

## 📁 Repository Structure
```text
Escape-From-Phobos/
├── Controller.py                          # V10 Ultra-Stable High-Score GNC Controller (Team Stella)
├── BasicController.py                     # Stock STU4-Solution reference controller (unmodified)
├── UserAPI.py                             # Stock STU4-Solution API reference utilities (unmodified)
├── requirements.txt                       # Python dependencies (zero external pip packages required)
├── README.md                              # Project documentation, benchmarks & validation instructions
├── .gitattributes                         # Stock STU4-Solution Git LFS & attribute rules
├── .gitignore                             # Python + SpaceTeamsPRO/STU4-Solution ignore rules
├── docs/
│   └── SOLUTION_WRITEUP.md                # Full technical write-up for Final Submission
└── submission/
    ├── SUBMISSION_FORM_DATA.txt           # Verified Run 1 / Run 2 telemetry, averages, hashes & UUIDs
    └── logs/
        ├── 261121_014614_COMPETITION_SIM__45f/  # Full unmodified STPro log folder (Run 1 — Score 53.6)
        └── 261121_014924_COMPETITION_SIM__a01/  # Full unmodified STPro log folder (Run 2 — Score 55.1)
```

---

## 🛠️ Instructions to Run & Validate
1. Install **SpaceTeamsPRO** (v0.55.0 or v0.56.0) and clone `https://github.com/SimDynamX/STU4-Sim.git` (`commit 0d753a2`, `Backend Sim Version: 2026/11/11`).
2. Copy `Controller.py` from this repository into `Assets/Repos/STU4-Solution/Controller.py` (or clone this repository directly as your active `STU4-Solution` repo). No `pip install` steps are needed.
3. Launch **SpaceCRAFT.exe** and open `COMPETITION_SIM.simconf` or `FINAL_SUBMISSION.simconf` from `Repos/STU4-Sim/Sims/` (both share identical physics, `Backend Sim Version: 2026/11/11`, and `seed = 8193`).
4. Press **Play** to run the simulation.

---

## 🏷️ Release Tags
- **`v2.0-final-submission`**: Verified Final Submission (V10 Controller, `Backend Sim Version: 2026/11/11`, Scores **53.6** & **55.1**, Avg **54.35**, **0.0 J** damage).
- **`v1.0-submission-verified`**: Verified Finals Qualification Submission (Scores **48.8** & **48.7**, Avg **48.75**, **0.0 J** damage).
