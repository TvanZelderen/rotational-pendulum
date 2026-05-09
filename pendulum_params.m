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
%   A3. Motor model: tau = km * u  (back-EMF and inductance neglected).
%   A4. Damping is purely viscous (linear). Real system likely has Coulomb
%       friction too — revisit during system identification.
%   A5. Sensor bias: each angle output has a constant offset (hardware removes
%       this with a Simulink constant block). Offsets listed below.

%% Arm 1  (c1 — motor to joint)
p.l1   = 0.10;                         % length of arm 1, pivot to joint [m]  (measured)
p.m1   = 0.25;                         % mass of arm 1 [kg]  — pre-ID estimate; TODO: identify
p.lc1  = p.l1 / 2;                     % CoM distance from pivot [m]  (uniform rod assumption)
p.J1   = (1/3) * p.m1 * p.l1^2;       % moment of inertia about pivot [kg·m²]  (uniform rod)
p.c1   = 1e-2;                         % viscous damping at joint 1 [N·m·s/rad]  — see A4
                                        % TODO: identify c1/J1 composite from arm 1 experiment

%% Arm 2 / pendulum  (c2 — joint to ball)  — see assumption A1
p.l2   = 0.099404;                     % length of arm 2, joint to ball [m]  — identified via beta=g/l2
p.m2   = 0.024;                        % tip mass [kg]  — pre-ID estimate; TODO: identify
p.J2   = p.m2 * p.l2^2;               % point mass inertia about joint [kg·m²]

% Identified composites (free-swing experiment, 2 runs averaged, 2026-05-08):
%   Run 1: alpha = 0.325573, beta = 98.729513, l2 = 0.099362
%   Run 2: alpha = 0.308600, beta = 98.646966, l2 = 0.099446
p.alpha2 = 0.317087;                   % c2 / (m2 * l2^2)  [rad/s] — identified via nlgreyest
p.c2     = p.alpha2 * p.m2 * p.l2^2;  % derived from alpha2 and current m2 estimate
                                        % NOTE: c2 will change if m2 is revised — alpha2 is the
                                        % true identified quantity, not c2 individually

%% Motor / drive  — see assumption A3 (partially relaxed)
% NOTE: input u is scaled to [-1, +1], NOT raw volts.
% km represents peak torque [N·m per normalised unit].
% Back-EMF braking: tau_bemf = kb * dth1, where kb = kt*ke/R.
% Inductance still neglected (A3 applies to inductance only now).
p.km   = 0.1;                          % pre-ID estimate; TODO: identify from arm 1 experiment
p.kb   = 0.2;                         % back-EMF damping [N·m·s/rad] — pre-ID estimate; TODO: identify

%% Environment
p.g    = 9.81;                         % gravitational acceleration [m/s²]

%% Sensor noise  (used by run_sim.m to mimic real sensor behaviour)
p.noise_std_th1 = 0.01;               % std dev of theta1 measurement noise [deg]
p.noise_std_th2 = 0.01;               % std dev of theta2 measurement noise [deg]
                                       % TODO: estimate from hardware data (variance at rest)

%% Sensor bias offsets
% Hardware calibration derived 2026-05-01:
%   theta1: raw mean = -3.7842,  gain = 360/4.9036  (≈ 73.4 deg/raw-unit)
%   theta2: raw mean = -1.2037,  gain = 360/4.9390  (≈ 72.9 deg/raw-unit)
% Applied in hwinit.m and rotpentemplate.slx.
% Simulation outputs calibrated degrees directly — no correction needed here.
p.bias_th1 = 0.0;                     % [deg]
p.bias_th2 = 0.0;                     % [deg]

%% Angle convention  (matches rotpentemplate.slx output)
% theta1 = 0 : arm 1 hanging straight down  (stable equilibrium)
% theta2 = 0 : arm 2 aligned with downward extension of arm 1
% Inertial angle of arm 2 = theta1 + theta2
% Radians inside ODE; degrees in simout.
