% Physical parameters for the DCSC rotational pendulum.
% Notation matches the diagram: two arms (c1, c2) swinging in a vertical plane,
% driven by a motor at the base whose axis is horizontal.
%
% Run this script to populate struct p in your workspace,
% then pass it to rotpen_ode.m.
%
% CONVENTIONS:
%   u        : normalised motor input, u in [-1, +1].  NOT raw volts.
%   km       : peak torque [N·m per normalised unit].
%   th2      : RELATIVE angle — arm 2 relative to arm 1 extension.
%              Inertial angle of arm 2 = th1 + th2.  Down-down = (0, 0).
%   Fields   : snake_case throughout (e.g. p.bias_th1, p.lc1).
%              Use km (not kt), J1/J2 (not I1/I2), l1/l2 (not L1/L2).
%
% MODELLING ASSUMPTIONS (grey-box — update after system identification):
%   A1. Arm 2 is a point mass m2 at the tip (arm c2 itself is massless).
%   A2. Arm 1 is stiff: reaction forces from arm 2 onto arm 1 are negligible,
%       so the motor torque drives arm 1 independently (coupling term in arm 1
%       equation of motion is small and can be dropped after validation).
%   A3. Motor model: tau = km * u - kb * dth1  (back-EMF included;
%       inductance neglected).
%   A4. Damping is purely viscous (linear). Real system likely has Coulomb
%       friction too — revisit during system identification.
%   A5. Sensor bias: each angle output has a constant offset (hardware removes
%       this with a Simulink constant block). Offsets listed below.

%% Arm 1  (c1 — motor to joint)
p.l1   = 0.10;                         % length of arm 1, pivot to joint [m]  (measured)
p.m1   = 0.25;                         % mass of arm 1 [kg]  — pre-ID estimate; TODO: identify
p.lc1  = p.l1 / 2;                     % CoM distance from pivot [m]  (uniform rod assumption)
p.J1   = (1/3) * p.m1 * p.l1^2;       % moment of inertia about pivot [kg·m²]  (uniform rod)

%% Arm 2 / pendulum  (c2 — joint to ball)  — see assumption A1
p.l2   = 0.086841;                     % length of arm 2, joint to ball [m]  — identified via beta=g/l2
p.m2   = 0.024;                        % tip mass [kg]  — placeholder; m2 NOT independently
                                        % identifiable from free-swing data alone.
                                        % TODO: confirm with additional free-swing runs.
p.J2   = p.m2 * p.l2^2;               % point mass inertia about joint [kg·m²]

% Identified composites (free-swing experiment, latest averaged run, 2026-05-12):
%   alpha2 = c2 / (m2 * l2^2) = 0.164111  [1/s]
%   beta   = g / l2            = 112.964563  ->  l2 = 0.086841 m
% NOTE: alpha2 is the true identified quantity.  c2 is derived and will
%       change if m2 is revised — do not treat c2 as independently identified.
p.alpha2 = 0.164111;                   % c2 / (m2 * l2^2)  [1/s] — identified via nlgreyest
p.c2     = p.alpha2 * p.m2 * p.l2^2;  % derived from alpha2 and current m2 estimate

%% Motor / drive  — see assumption A3 (partially relaxed)
% NOTE: input u is scaled to [-1, +1], NOT raw volts.
% km represents peak torque [N·m per normalised unit].
% Back-EMF braking: tau_bemf = kb * dth1, where kb = kt*ke/R.
% Inductance still neglected (A3 applies to inductance only now).
p.km   = 18.271125;                    % peak motor torque [N·m per normalised unit] — sysid_arm1_ramp 2026-05-18
p.kbc1 = 2.502519;                     % composite: kb (back-EMF) + c1 (joint damping) [N·m·s/rad]
                                       % identified via terminal-velocity regression 2026-05-18
p.tauc_static  = 1.772299;            % Coulomb breakaway torque [N·m] — sysid_arm1_ramp 2026-05-18
p.tauc_kinetic = 1.291164;            % Coulomb kinetic torque [N·m]   — sysid_arm1_termvel 2026-05-18

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
