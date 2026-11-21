| [Rules & Info (#218)](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/218) | [Leaderboard (#247)](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/247) | [Starter Code Repo](https://github.com/SimDynamX/STU4-Solution) | [STU4 Wiki](https://github.com/SimDynamX/STU4-Solution/wiki) | [Forum](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/categories/2026-escape-from-phobos) |
| --- | --- | --- | --- | --- |

# 🚀 Escape from Phobos — Team Stella (STU4 Final Submission)

Official solution repository for **Team Stella** competing in **SpaceTeams University #4 (STU4) — Fall 2026: "Escape from Phobos"** on SpaceTeamsPRO v0.55.0 (`SimDynamX/STU4-Sim` commit `0d753a2`, `Backend Sim Version: 2026/11/11`).

---

## 🛰️ Mission & Challenge Overview
Following an asteroid impact on a mining base on Phobos, our autonomous controller pilots an evacuation shuttle (*SpaceBuggy*, mass $3388.7\text{ kg}$, 38 RCS jets) through a dense, dynamically moving 500-rock 3D debris field in Phobos' low-gravity environment to rendezvous with the orbiting *AsteroidCycler* keep-out sphere ($\le 50\text{ m}$).

The objective is to maximize the competition score—optimizing both **flight time** and **propellant efficiency**—while achieving **zero collisions** ($0.0\text{ J}$ damage) consistently across two consecutive runs.

---

## 🏆 Final Submission Performance (V11 — Two Consecutive `FINAL_SUBMISSION` Runs)

Two consecutive live `FINAL_SUBMISSION.simconf` runs (`Backend Sim Version: 2026/11/11`, `seed = 8193`, unmodified MD5 `B109312681B5518D57B36B9FD25DC120`) were recorded back-to-back in a single continuous screen recording (`VIDEO.mp4`, `6m 29.9s`) using the exact `Controller.py` in this repository (`MD5: 09008B3222FD2389E459E0000CE31E1A`, `SHA256: CF0D11EF2678348C640FDB0E7A8B9C992141FEEFA865DF93B1ED63BA680AF521`):

| Metric | Run 1 (`61e`) | Run 2 (`d04`) | **2-Run Average / Combined** |
|---|:---:|:---:|:---:|
| **Final Score** | **54.3** | **60.6** | **57.45** *(+3.10 pts vs V10)* |
| **Sim Elapsed Time (SET)** | $179.73\text{ s}$ | $175.40\text{ s}$ | **$177.57\text{ s}$** |
| **Total Fuel Used** | $46.84\text{ kg}$ | $43.35\text{ kg}$ | **$45.10\text{ kg}$** *(-3.27 kg vs V10)* |
| **Total Damage Taken** | $0.0\text{ J}$ | $0.0\text{ J}$ | **$0.0\text{ J}$ (Zero Collisions)** |
| **Final Distance to Ship** | $49.53\text{ m}$ | $49.98\text{ m}$ | **Within Keep-Out ($\le 50.0\text{ m}$)** |
| **Run UUID** | `61ea859f-4098-687b-1bb3-c4a6ba9bfc05` | `d04311e8-4114-7772-d6d9-5eb0374ae05f` | Verified (`submission/logs/`) |

Across all **7 completed live V11 flights** in SpaceCRAFT (6 official `FINAL_SUBMISSION` runs + 1 `COMPETITION_SIM` run), V11 achieved **7/7 (100%) zero-damage escapes** with peak scores of **62.9** (`40.91 kg`, `COMPETITION_SIM`), **61.8** (`42.64 kg`, `FINAL_SUBMISSION`), **60.6** (`43.35 kg`, `FINAL_SUBMISSION`), and **60.5** (`43.29 kg`, `FINAL_SUBMISSION`).

Full unmodified STPro simulation logs for both consecutive recorded runs are included under [`submission/logs/`](submission/logs/), and the complete technical report is in [`docs/SOLUTION_WRITEUP.md`](docs/SOLUTION_WRITEUP.md).

---

## 🧠 V11 Guidance, Navigation & Control Architecture

1. **Lyapunov Multi-Vertex Pure-Axis RCS Allocator (100% L1 Efficiency):**
   - Computes exact zero-torque 4-jet simplex vertices on `+Z`, `+X`, `-X`, `-Y`, `-Z` ($0.1929\text{ m/s}^2$, $2.01\times$ default `+Z` thrust) and 3-jet vertices on `+Y` ($0.1447\text{ m/s}^2$), automatically synced live from `RCSConfig`, `getMass()`, and `getInertia()`.
   - Uses closed-loop Lyapunov feedback on measured body angular rates (`ang_vel_body`) to cancel residual torque to near zero ($\|\omega\| \approx 0.005\text{–}0.015\text{ rad/s}$) and steer the gravity-compensated launch aim at $V_{\text{cruise}} = 9.15\text{ m/s}$ with zero extra propellant.

2. **Zero-Fuel `CLIMB_YIELD` Timing Modulation (V11 Breakthrough):**
   - When a fast-crossing rock intersects the shuttle's projected `CLIMB_Z` ascent corridor, instead of burning lateral propellant to dodge or tilting off-axis, V11 briefly pauses `+Z` thrust (`CLIMB_YIELD`, $0.00\text{ kg/s}$ fuel, $0.0^\circ$ tilt, bounded by `MAX_YIELD_S = 10.5 s` with chatter-free hysteresis pad `YIELD_CLEAR_PAD = 3.5 m` and neighbor safety checks) so the obstacle passes harmlessly ahead before resuming climb.

3. **Tumble-Immune 500-Rock Moving-Debris Tracker & 2-Sweep Bounce Adaptation:**
   - Tracks all 500 radar contacts across SpaceCRAFT's $4.17\text{ s}$ round-robin radar sweep using gravity-compensated exponentially-weighted 7-sweep WLS ($\tau = 14\text{ s}$, $\lambda_v = 5.0$) fused with Doppler range-rate measurements and exact timestamp-aligned shuttle state history.
   - Adapts rapidly to confirmed 2-sweep rock-rock collisions on small/medium debris ($R < 17.8\text{ m}$) while preserving 7-sweep smoothing on giant tumbling boulders ($R \ge 17.8\text{ m}$) and requiring $n \ge 5$ observations for high-energy monster standoffs (`CHAOS_HI_MAX = 16.5 m`) beyond `21.5 s` CPA (`CHAOS_HI_N5_TCA = 21.5 s`) to eliminate unconverged false evades.

4. **Simpson Midpoint Gravity, Online Coast Observer & Exact 3D CPA Edge ZEM:**
   - Propagates ballistic trajectories using Simpson midpoint Phobos gravity plus an online coast environmental acceleration observer (`a_env_extra`) to capture unmodeled perturbations.
   - Evaluates exact 3D CPA Zero-Effort-Miss (`norm(zem_vec)`) targeting the fuel-optimal inner edge of the $50\text{ m}$ capture sphere (`44.0 / 42.5 / 41.0 m` stop band) with anti-chatter `0.50 s` axis hysteresis, `23 s` directional debris lookahead, and per-threat opposite-axis locks.
   - Transitions to **Zero-Waste Hysteresis Coast Damping** (`COAST_DAMP_ENTER = 0.024 rad/s`, `EXIT = 0.008 rad/s`), cruising ballistically for >100 seconds at $0.00\text{ kg/s}$ fuel (`0.274 ms/tick` CPU latency at 60 Hz) until thruster cutoff inside `49.2 m`.

See [`docs/SOLUTION_WRITEUP.md`](docs/SOLUTION_WRITEUP.md) for the complete algorithmic specification and flight-log telemetry breakdown.

---

## ✅ Solution Validity & Verification Summary

- **No Sim Modifications:** 100% unmodified `SimDynamX/STU4-Sim` (`commit 0d753a2`, `Backend Sim Version: 2026/11/11`, `FINAL_SUBMISSION.simconf` MD5 `B109312681B5518D57B36B9FD25DC120`).
- **Official APIs Only:** Uses only the on-board radar (`Radar/SortedDebrisData`), shuttle/target navigation state, and thruster commands via `SetCommand()`.
- **Zero External Pip Dependencies:** Runs out of the box using only SpaceTeamsPRO's built-in Python 3.12 stdlib, `spaceteams`, and `numpy` (see [`requirements.txt`](requirements.txt)).
- **6-DOF Offline Stress-Test Suite — 71 / 71 PASS (100.0%, `0.0 J` Damage, `0.274 ms/tick`):**
  - **10/10 Live-Reconstructed `45f` Runs** (5x Phobos + 5x Phobos+Mars/Coriolis): **100% PASS** (Mean Score `56.4` / `54.6`, Peak `60.9`)
  - **10/10 Core Production Worlds** (bounces, tumbling boulders, corridors, moving/static fields): **100% PASS** (`WorldA = 65.2`, `Run1ce 35/45/55 @ 1.18x = 3/3`)
  - **20/20 Multi-Seed Stress Tests** (bounce, tumbling boulder +4.5–4.8 m, irregular-mesh worlds): **100% PASS**
  - **6/6 Adversarial Worlds** (+4.5 m boulder tumble): **100% PASS** (Mean Score `54.8`, Mean Fuel `48.1 kg`)
  - **25/25 Random Moving Worlds** (+4.5 m boulder tumble): **100% PASS**

---

## 📁 Repository Structure
```text
Escape-From-Phobos/
├── Controller.py                           # V11 Ultra-Stable High-Score GNC Controller (Team Stella)
├── BasicController.py                      # Stock STU4-Solution reference controller (unmodified)
├── UserAPI.py                              # Stock STU4-Solution API reference utilities (unmodified)
├── requirements.txt                        # Python dependencies (zero external pip packages required)
├── README.md                               # Project documentation, benchmarks & validation instructions
├── .gitattributes                          # Stock STU4-Solution Git LFS & attribute rules
├── .gitignore                              # Python + SpaceTeamsPRO/STU4-Solution ignore rules
├── docs/
│   └── SOLUTION_WRITEUP.md                 # Full technical write-up for Final Submission
└── submission/
    ├── SUBMISSION_FORM_DATA.txt            # Verified Run 1 / Run 2 telemetry, averages, hashes & UUIDs
    └── logs/
        ├── 261121_045917_FINAL_SUBMISSION__61e/  # Full unmodified STPro log folder (Run 1 — Score 54.3)
        └── 261121_050231_FINAL_SUBMISSION__d04/  # Full unmodified STPro log folder (Run 2 — Score 60.6)
```

---

## 🛠️ Instructions to Run & Validate
1. Install **SpaceTeamsPRO** (v0.55.0 or v0.56.0) and clone `https://github.com/SimDynamX/STU4-Sim.git` (`commit 0d753a2`, `Backend Sim Version: 2026/11/11`).
2. Copy `Controller.py` from this repository into `Assets/Repos/STU4-Solution/Controller.py` (or clone this repository directly as your active `STU4-Solution` repo). No `pip install` steps are needed.
3. Launch **SpaceCRAFT.exe** and open `FINAL_SUBMISSION.simconf` (or `COMPETITION_SIM.simconf`) from `Repos/STU4-Sim/Sims/` (`Backend Sim Version: 2026/11/11`, `seed = 8193`).
4. Press **Play** to run the simulation.

---

## 🏷️ Release Tags
- **`v3.0-final-submission`**: Verified Final Submission (V11 Controller, `FINAL_SUBMISSION.simconf`, `Backend Sim Version: 2026/11/11`, Scores **54.3** & **60.6**, Avg **57.45**, Avg Fuel **45.10 kg**, **0.0 J** damage).
- **`v2.0-final-submission`**: Prior Final Submission Baseline (V10 Controller, `COMPETITION_SIM.simconf`, Scores **53.6** & **55.1**, Avg **54.35**, **0.0 J** damage).
- **`v1.0-submission-verified`**: Verified Finals Qualification Submission (Scores **48.8** & **48.7**, Avg **48.75**, **0.0 J** damage).
