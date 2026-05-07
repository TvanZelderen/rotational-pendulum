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
| Fill M and rhs in `rotpen_ode.m` | 🔄 | EOM in hand; transcription next |
| Linearise at stable eq. (both down) | ⬜ | Needed for linear control design |
| Linearise at unstable eq. (arm 2 up) | ⬜ | Needed for swing-up / balance controller |
| Discretise | ⬜ | h = 0.01 s; ZOH or Tustin |

---

## System identification
| Task | Status | Notes |
|---|---|---|
| Sensor calibration (bias + gain) | ✅ | offsets and gains in hwinit.m (2026-05-01) |
| Geometry: l₁ | ⬜ | Measure with ruler |
| Geometry: l₂ | 🔄 | ≈ 0.10 m — confirm with ruler |
| Input signal design | 💬 | Multisine / PRBS / spikes discussed; not yet chosen |
| Collect open-loop data — arm 1 | ⬜ | |
| Collect free-swing data — arm 2 | ⬜ | Arm 1 stationary, manually displace arm 2 |
| Identify km/m₁, c₁/m₁ | ⬜ | From arm 1 transfer function |
| Identify l₂, c₂/m₂ | ⬜ | From arm 2 free-swing (ωn and decay envelope) |
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
EOM fully derived. Next session: link 2 free-swing identification on hardware.
Immediate code task: transcribe M, n, g into `rotpen_ode.m`.
Parallel: measure l₁ with a ruler and confirm l₂ ≈ 0.10 m.
