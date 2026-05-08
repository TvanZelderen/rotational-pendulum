# Rotational Pendulum — CLAUDE.md

**Role:** Act as an experienced mentor guiding the student Socratically.
This is a project for learning, not just credits — provide scaffolding and hints,
leave derivations and design decisions for the student to work through.

**Always read `STRUCTURE.md`** if it is not already in context. It contains
the full course description, control objectives, setup details, and report guide.

---

## Branch strategy
- `main`       — shared with teammate; hardware-only code, keep clean
- `simulation` — simulation framework; merge to main once validated

## Key files
| File | Purpose |
|---|---|
| `firstScript.m` | Main experiment script; `run_with_simulation` flag at top |
| `run_sim.m` | ODE-based simulation; produces same `simout` as `rotpentemplate.slx` |
| `rotpen_ode.m` | Equations of motion — student fills in `M` and `rhs` |
| `pendulum_params.m` | Physical parameters + noise/bias → struct `p` |
| `rotpentemplate.slx` | Simulink model (hardware path) |
| `STRUCTURE.md` | Course context, setup details, report guide |

## Mode switch
```matlab
run_with_simulation = true;   % false → real hardware
```
Two `if` blocks only in `firstScript.m`; everything else shared.

## Angle convention
- θ₁ = 0, θ₂ = 0 : both links hanging straight down
- θ₂ relative (angle between arms); inertial angle of arm 2 = θ₁ + θ₂
- Raw sensor zero ≈ 85° physical; bias subtraction gives down = 0 convention
- Radians inside ODE; degrees in `simout`

## Input scaling
`u ∈ [−1, +1]` (not raw volts). Motor model: `tau = km * u` where `km` has
units [N·m per normalised unit].  This is the course convention.

---

## Modelling approach: grey box

Physics-derived structure (Lagrangian EOM) + parameters fitted from data.

| | White box | Grey box (our approach) |
|---|---|---|
| Structure | First principles | First principles |
| Parameters | Calculated | Identified from experiments |
| Noise | — | Additive Gaussian |
| Bias | — | Constant per sensor (A5) |

### Parameters to identify
| Parameter | Description | Source |
|---|---|---|
| `p.km` | Peak motor torque [N·m] | Lab manual |
| `p.c1` | Joint 1 viscous damping | Free-decay experiment |
| `p.c2` | Joint 2 viscous damping | Free-decay experiment |
| `p.m1`, `p.l1`, `p.m2`, `p.l2` | Geometry and masses | Physical measurement |

---

## Modelling assumptions (A1–A5, labelled in code)

**A1 — Point mass:** Arm c2 massless; m2 at tip only. Revisit if system ID residuals are large.

**A2 — Stiff arm 1:** Reaction forces from arm 2 onto arm 1 negligible.
Full coupling kept in EOM; off-diagonal of M can be dropped after validation.

**A3 — Simplified motor:** `tau = km * u`. Back-EMF and inductance ignored.
Revisit if closed-loop bandwidth is unexpectedly limited.

**A4 — Viscous damping only:** `c * dtheta`. Real joints likely have Coulomb friction too.
Add Coulomb term if system ID fit is poor near zero velocity.

**A5 — Sensor bias:** Raw hardware has ~85° offset per sensor.
`p.bias_th1` / `p.bias_th2` in `pendulum_params.m` are placeholders — values TBD from lab.
`run_sim.m` adds them so simulated output matches raw hardware format.

---

## Student TODO list
1. Derive `M` and `rhs` in `rotpen_ode.m` (Lagrangian — hints in file)
2. Fill in `p.km` from lab manual; fill in `p.bias_th*` from lab setup
3. Validate: zero input, small `th2_0`; check arm 2 oscillates near ω₀ = √(g/l₂) ≈ 9.9 rad/s (l₂ ≈ 0.10 m)
4. System ID to fit `c1`, `c2`, verify `km`
5. Design controller(s); test in simulation before hardware
6. Compare sim vs hardware on identical `simin`
