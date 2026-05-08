clear; clc;
% Prerequisites (run once per session before this script):
%   1. calib.m  — opens fugiboard connection, resets encoder, activates relay
%   2. hwinit.m — sets sensor gain/offset calibration values

%% Parameters
h    = 0.01;  % sample period [s]
Tsim = 30;    % experiment duration [s]

%% Input signal — zero to hold arm 1 stationary
t     = (0:h:Tsim)';
u     = zeros(size(t));
simin = [t, u];

%% Run experiment
sim rotpentemplate

%% Extract phi = th1 + th2 (Simulink output 5), zero at equilibrium
y   = simout.Data;           % N x 5: [th1, dth1, th2, dth2, phi] in degrees
phi = y(:,5);
phi = mod(phi - mean(phi(end-100:end)) + 180, 360) - 180;

%% Overview plot — use to set trimStart / trimEnd
figure;
plot(t, phi)
xlabel('Time [s]'); ylabel('\phi [deg]')

%% Trim start and end
trimStart   = 2;   % [s] — skip initial impulse
trimEnd     = 24;    % [s] — cut settled tail

iStart      = trimStart * 100 + 1;
iEnd        = numel(t) - (Tsim - trimEnd) * 100;
phi_trimmed = phi(iStart:iEnd);
tTrimmed    = t(iStart:iEnd);

%% Convert to radians (nonlinear ODE uses sin)
phi_rad = deg2rad(phi_trimmed);

%% idnlgrey — nonlinear grey-box identification
data = iddata(phi_rad, [], h);

alpha0 = 1;
beta0  = 9.81 / 0.1;
x0     = [phi_rad(1); 0];

sys = idnlgrey('link2_ode', [1, 0, 2], {alpha0; beta0}, x0, 0);
sys.InitialStates(1).Fixed = false;
sys.InitialStates(2).Fixed = false;

opt = nlgreyestOptions('Display', 'on');
sys_est = nlgreyest(data, sys, opt);

fprintf('alpha (c2/m2/l2^2) = %.6f\n', sys_est.Parameters(1).Value);
fprintf('beta  (g/l2)       = %.6f\n', sys_est.Parameters(2).Value);
fprintf('l2                 = %.6f m\n', 9.81 / sys_est.Parameters(2).Value);

%% Results

% alpha (c2/m2/l2^2) = 0.325573
% beta  (g/l2)       = 98.729513
% l2                 = 0.099362 m

% alpha (c2/m2/l2^2) = 0.308600
% beta  (g/l2)       = 98.646966
% l2                 = 0.099446 m