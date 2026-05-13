# Simulation Guide

How the simulation fits alongside the real hardware workflow.

---

## What we built

| File | Purpose |
|---|---|
| `pendulum_params.m` | Physical constants → struct `p` |
| `furuta_ode.m` | Equations of motion (yours to complete) |
| `run_sim.m` | ODE integrator, produces same `simout` as `rotpentemplate.slx` |

`run_sim.m` is a **drop-in** for the hardware path.  Once `furuta_ode.m` is filled in, you can run `run_open_loop.m` (from the "Extract outputs" section) on simulation output without touching the hardware.

---

## Workflow comparison

| Step | Hardware path | Simulation path |
|---|---|---|
| Session init | `calib.m` → `hwinit.m` | `pendulum_params.m` |
| Set input | `simin = [t, u]` | same |
| Run | `sim rotpentemplate` | `run_sim` |
| Read output | `simout.Data` | same |

---

## Optional: hybrid Simulink model

If you want to keep a Simulink diagram (e.g. to wire a controller visually), you can create `rotpentemplate_sim.slx` that replaces the hardware S-function blocks with a MATLAB Function block calling `furuta_ode`.

### Blocks to place

1. **From Workspace** — reads `simin`; set *Variable name* = `simin`, *Sample time* = `h`
2. **MATLAB Function** block — replaces `sfusbin` + `sfusbout` + the physical plant:
   - Input port: `u` (voltage from your controller or from simin)
   - Output port: `y` (2×1 vector: [theta1_deg; theta2_deg])
   - Inside the function: call `furuta_ode`, integrate one step, convert to degrees
   - You will need to store state between steps → declare the state vector as a **persistent** variable
3. **To Workspace** — saves output as Timeseries named `simout`; set *Save format* = `Timeseries`
4. Wire: From Workspace → (your controller) → MATLAB Function → To Workspace

### Persistent state pattern (inside the MATLAB Function block)

```matlab
function y = pendulum_step(u, p, h)
    persistent x;
    if isempty(x)
        x = [0; 0; 0.1; 0];   % initial condition — match run_sim.m
    end
    % one Euler step (replace with RK4 for accuracy)
    x = x + h * furuta_ode(0, x, u, p);
    y = rad2deg([x(1); x(3)]);
end
```

> A fixed-step Euler integrator is fine for quick tests at `h = 0.01 s`, but
> note that `run_sim.m` uses `ode45` which is more accurate for validating your EOM.

---

## Suggested order of work

1. **Derive the EOM** — fill in `M` and `rhs` in `furuta_ode.m`
   - Check: with `km = 0` and a small `th2_0`, does the pendulum oscillate at the right natural frequency?
   - Natural frequency (small angle): `ω₀ = sqrt(g / l2)`

2. **Fill in missing parameters** — `km`, `b1`, `b2` in `pendulum_params.m`

3. **Validate open-loop** — run `run_sim.m` with zero input; observe free swing

4. **Design a controller** — wire it between `simin` and `run_sim.m` (or in the Simulink model)

5. **Compare sim vs hardware** — run the same `simin` on both paths, overlay the plots
