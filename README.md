# 🚀 Escape from Phobos — Team Stella (STU4)

Official repository for **Team Stella** participating in the **SpaceTeams University #4 (STU4) — "Escape from Phobos"** competition on SpaceTeamsPRO v0.55.0.

---

## 🛰️ Mission & Challenge Overview
Following an asteroid impact on a mining base on Phobos, our autonomous controller pilots an evacuation shuttle through a dense 3D debris field in low gravity to rendezvous with the orbiting *AsteroidCycler*.

The objective is to optimize flight time and fuel consumption while maintaining zero collisions ($0.0\text{ J}$ damage).

---

## 🏆 Flight Performance & Benchmarks (Finals Qualification)

Two consecutive evaluation runs were conducted using this identical codebase, meeting all qualification requirements:

| Metric | Run 1 | Run 2 | **Average / Combined** |
|---|:---:|:---:|:---:|
| **Score** | **49.7** | **48.8** | **49.3** (49.25) |
| **Sim Elapsed Time (SET)** | $219.21\text{ s}$ | $219.68\text{ s}$ | **$219.45\text{ s}$** |
| **Total Fuel Used** | $41.26\text{ kg}$ | $41.87\text{ kg}$ | **$41.57\text{ kg}$** |
| **Total Damage Taken** | $0.0\text{ J}$ | $0.0\text{ J}$ | **$0.0\text{ J}$ (Zero Collisions)** |
| **Final Distance to Ship** | $49.57\text{ m}$ | $49.60\text{ m}$ | **Within Keep-Out ($\le 50\text{ m}$)** |
| **Run UUID** | `5e48146f-427f-ac85-aa39-4bb4d8fd385c` | `cbb437f6-47af-3929-5489-aea7092169a9` | Verified |

---

## 🧠 Guidance, Navigation & Control Architecture

1. **Direct Line-of-Sight (LOS) Target Guidance:**
   - Computes unit vector $\hat{u}_{\text{goal}}$ from the shuttle towards the *AsteroidCycler*.
   - Targets a calibrated cruise speed of $7.85\text{ m/s}$ (with cutoff at $7.70\text{ m/s}$) to overcome Phobos' gravity well and reach the target bubble cleanly.

2. **Predictive Radar Collision Avoidance:**
   - Evaluates the nearest radar obstacles using relative velocity closing rate ($v_{\text{closing}} > 0$).
   - Calculates closest approach distance $d_{\text{min}}$ and time-to-closest-approach $t_{\text{cpa}}$ within a predictive lookahead horizon.
   - Enforces a calibrated safe bubble ($r_{\text{safe}} = \text{radius} + 5.0\text{ m}$), applying strong orthogonal repulsive acceleration when threats enter the collision cone.

3. **Fuel-Optimal Terminal Coasting:**
   - Disables continuous drift chattering once cruise velocity is acquired.
   - Allows the vehicle to coast ballistically across the final kilometer, preserving fuel and preventing reactive steering oscillations near debris.

---

## 📁 Repository Structure
```
Escape-From-Phobos/
├── Controller.py          # Complete autonomous flight controller script (Team Stella)
├── BasicController.py     # Baseline reference controller
├── UserAPI.py             # SpaceTeams API reference utilities and examples
├── README.md              # Project documentation and performance benchmarks
├── .gitattributes         # Git LFS & line ending attributes
└── .gitignore             # Standard Python / simulation gitignore
```

---

## 🛠️ Instructions to Run
1. Launch **SpaceCRAFT.exe** (SpaceTeamsPRO v0.55.0).
2. Open `COMPETITION_SIM.simconf` from `Repos/STU4-Sim/Sims/`.
3. Ensure this `Controller.py` is loaded in your active solution repo.
4. Press Play to run the simulation.
