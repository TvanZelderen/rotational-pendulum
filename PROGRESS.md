# Project Progress

Legend: ✅ Done · 🔄 In progress · 💬 In discussion · ⬜ Not started

---

## Simulation framework
| Task | Status | Notes |
|---|---|---|
| Branch `simulation` off `main` | ✅ | Teammate's workflow unaffected |
| `pendulum_params.m` scaffold | ✅ | l2 ≈ 0.10 m (ruler); calibration values in hwinit.m |
| `run_sim.m` ODE runner | ✅ | Same `simout` format as hardware |
| `rotpen_ode.m` scaffold | ✅ | M and rhs left for student to fill |
| `firstScript.m` mode switch | ✅ | `run_with_simulation` flag |
| **Derive EOM — full system** | ✅ | M, n, g, τ derived; ready to fill rotpen_ode.m |

---

## Modelling
| Task | Status | Notes |
|---|---|---|
| EOM — arm 1 (driven) | ✅ | Identifiable composites: km/m₁, c₁/m₁ |
| EOM — arm 2 (passive) | ✅ | Identifiable composites: l₂ from ωn, c₂/m₂l₂² from ζ |
| Full coupled EOM (Lagrangian) | ✅ | M, n, g derived — see STRUCTURE.md |
| Fill M and rhs in `rotpen_ode.m` | 🔄 | M scaffold open; ready to fill entries next session |
| Linearise at stable eq. (both down) | ⬜ | Needed for linear control design |
| Linearise at unstable eq. (arm 2 up) | ⬜ | Needed for swing-up / balance controller |
| Discretise | ⬜ | h = 0.01 s; ZOH or Tustin |

---

## System identification
| Task | Status | Notes |
|---|---|---|
| Sensor calibration (bias + gain) | ✅ | offsets and gains in hwinit.m (2026-05-01) |
| Geometry: l₁ | ✅ | 0.10 m measured |
| Geometry: l₂ | ✅ | l_eff = 0.0868 m (identified); physical 0.10 m — A1 (point mass) only approximate |
| Input signal design | 💬 | Multisine / PRBS / spikes discussed; not yet chosen |
| Collect open-loop data — arm 1 | ⬜ | |
| Collect free-swing data — arm 2 | ✅ | Done 2026-05-08; re-run planned 2026-05-12 to average |
| Identify km, kb, c₁ | 💬 | km from lab manual; kb (back-EMF braking) + c₁ from arm 1 free-decay (u=0); separate experiment with u≠0 to isolate km from inertia |
| Identify l₂, c₂/m₂l₂² | 🔄 | α=0.1627, β=113.06, l_eff=0.0868 m — 90% fit (2026-05-11); averaging runs tomorrow |
| Identify coupled parameters | ⬜ | May need m₂ separately — see notes |
| Validate on held-out dataset | ⬜ | Course requirement |
| Closed-loop sysid (unstable eq.) | ⬜ | Required for upright balance; do last |

---

## Observer
| Task | Status | Notes |
|---|---|---|
| Decide on observer type (Luenberger / Kalman) | ⬜ | States: θ₁, θ̇₁, θ₂, θ̇₂; outputs: θ₁, θ₂ only |
| Design observer in simulation | ⬜ | |
| Validate observer | ⬜ | |

---

## Control design
| Task | Status | Notes |
|---|---|---|
| Choose 2 linear methods | 💬 | Pole placement and LQR are natural candidates |
| (Optional) 1 nonlinear method | 💬 | Swing-up — energy-based or NDI |
| Controller 1 — stable eq. (both down) | ⬜ | Easier starting point |
| Controller 2 — unstable eq. (arm 2 up) | ⬜ | Requires linearisation at θ₂ = 180° |
| Test controllers in simulation | ⬜ | |
| Test controllers on hardware | ⬜ | |
| Compare performance | ⬜ | Robustness, disturbance rejection, steady-state error |

---

## Report
| Task | Status | Notes |
|---|---|---|
| Setup description + conventions | ⬜ | |
| Modelling section | ⬜ | |
| System ID section | ⬜ | |
| Control design section | ⬜ | |
| Discussion | ⬜ | |

---

## Where we are right now
Link 2 free-swing ID done — 90% fit after aligning trim to first natural peak (IC fix).
l_eff = 0.0868 m identified (point-mass A1 approximate; distributed mass shifts effective length).
Next session: collect more free-swing runs to average α, β. Then arm 1 ID: free-decay (u=0) to get (c₁+kb)/I₁, driven experiment to isolate km. Back-EMF term τ = km·u − kb·dθ₁ not yet in rotpen_ode.m — add after basic EOM is transcribed.
