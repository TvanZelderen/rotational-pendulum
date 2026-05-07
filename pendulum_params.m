% Physical parameters for the DCSC rotational pendulum.
% Notation matches the diagram: two arms (c1, c2) swinging in a vertical plane,
% driven by a motor at the base whose axis is horizontal.
%
% Run this script to populate struct p in your workspace,
% then pass it to rotpen_ode.m.
%
% MODELLING ASSUMPTIONS (grey-box — update after system identification):
%   A1. Arm 2 is a point mass m2 at the tip (arm c2 itself is massless).
%   A2. Arm 1 is stiff: reaction forces from arm 2 onto arm 1 are negligible,
%       so the motor torque drives arm 1 independently (coupling term in arm 1
%       equation of motion is small and can be dropped after validation).
%   A3. Motor model: tau = km * V  (back-EMF and inductance neglected).
%   A4. Damping is purely viscous (linear). Real system likely has Coulomb
%       friction too — revisit during system identification.
%   A5. Sensor bias: each angle output has a constant offset (hardware removes
%       this with a Simulink constant block). Offsets listed below; update when
%       provided from the lab setup.

%% Arm 1  (c1 — motor to joint)
p.l1   = 0.215;        % length of arm 1, pivot to joint [m]
p.m1   = 0.095;        % mass of arm 1 [kg]
p.lc1  = p.l1 / 2;    % distance from pivot to CoM of arm 1 [m]  (uniform rod)
p.J1   = (1/3) * p.m1 * p.l1^2;   % moment of inertia of arm 1 about pivot [kg·m²]
p.c1   = 1e-3;         % viscous damping at joint 1 [N·m·s/rad]  — see A4
                       % TODO: identify c1 from free-decay experiment (system ID)

%% Arm 2 / pendulum  (c2 — joint to ball)  — see assumption A1
p.l2   = 0.10;         % length of arm 2, joint to ball [m]  (≈10 cm, measured by ruler — TODO: verify)
p.m2   = 0.024;        % mass of ball (tip mass) [kg]  — arm c2 itself treated as massless
p.c2   = 5e-5;         % viscous damping at joint 2 [N·m·s/rad]  — see A4
                       % TODO: identify c2 from free-decay experiment (system ID)

%% Motor / drive  — see assumption A3
% NOTE: the course scales input u to [-1, +1], NOT raw volts.
% km therefore represents peak torque in [N·m per normalised unit].
p.km   = 0.0;          % peak motor torque [N·m]  (tau = km * u,  u in [-1, 1])
                       % TODO: look up km in the lab manual

%% Environment
p.g    = 9.81;         % gravitational acceleration [m/s²]

%% Sensor noise  (used by run_sim.m to mimic real sensor behaviour)
p.noise_std_th1 = 0.01;   % std dev of theta1 measurement noise [deg]
p.noise_std_th2 = 0.01;   % std dev of theta2 measurement noise [deg]
                           % TODO: estimate from hardware data (variance at rest)

%% Sensor bias offsets
% Hardware raw values (from hwinit.m, derived 2025-05-01):
%   theta1: raw mean = -3.7842,  gain = 360/4.9036  (≈ 73.4 deg/raw-unit)
%   theta2: raw mean = -1.2037,  gain = 360/4.9390  (≈ 72.9 deg/raw-unit)
%
% The simulation computes true physics angles and outputs calibrated degrees
% directly, so no bias correction is needed here.  Set to non-zero only if
% you want run_sim to mimic raw (un-calibrated) hardware output for testing
% the calibration pipeline itself.
p.bias_th1 = 0.0;    % [deg]
p.bias_th2 = 0.0;    % [deg]

%% Angle convention  (matches rotpentemplate.slx output)
% theta1 = 0 : arm 1 hanging straight down  (stable equilibrium)
% theta2 = 0 : arm 2 aligned with the downward extension of arm 1
%              (i.e. relative angle at joint is zero)
% Inertial (absolute) angle of arm 2 = theta1 + theta2  (simple geometry)
% Both in RADIANS inside the ODE; converted to DEGREES for simout.
