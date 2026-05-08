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

%% Extract and wrap angles
y   = simout.Data;           % N x 4: [th1, dth1, th2, dth2] in degrees
th1 = mod(y(:,1) + 180, 360) - 180;
th2 = mod(y(:,3) + 180, 360) - 180;

%% Cut start: remove initial impulse
trimStart   = 1.5;           % [s]
iStart      = round(trimStart / h) + 1;
th1_cut     = th1(iStart:end);
th2_cut     = th2(iStart:end);
t_cut       = t(iStart:end);

%% Re-centre: subtract mean th1 so th2 equilibrium sits at 0
th2_centred = th2_cut - mean(th1_cut);

%% Mod back to (-180, 180] to remove wrapping artifacts
th2_centred = mod(th2_centred + 180, 360) - 180;

%% Cut tail: now that equilibrium is at 0, threshold works cleanly
activeThresh = 2;            % [deg]
iEnd         = find(abs(th2_centred) > activeThresh, 1, 'last');
th2_trimmed  = th2_centred(1:iEnd);
tTrimmed     = t_cut(1:iEnd);

%% Plot trimmed data
figure;
plot(tTrimmed, th2_trimmed)
xlabel('Time [s]'); ylabel('\theta_2 [deg]')
title('Link 2 — trimmed data for identification')

%% Convert to radians (nonlinear ODE uses sin)
th2_rad = deg2rad(th2_trimmed);

%% idnlgrey — nonlinear grey-box identification
data = iddata(th2_rad, [], h);

alpha0 = 1;
beta0  = 9.81 / 0.1;
x0     = [th2_rad(1); 0];

sys = idnlgrey('link2_ode', [1, 0, 2], {alpha0; beta0}, x0, 0);
sys.InitialStates(1).Fixed = false;
sys.InitialStates(2).Fixed = false;

opt = nlgreyestOptions('Display', 'on');
sys_est = nlgreyest(data, sys, opt);

fprintf('alpha (c2/m2/l2^2) = %.6f\n', sys_est.Parameters(1).Value);
fprintf('beta  (g/l2)       = %.6f\n', sys_est.Parameters(2).Value);
fprintf('l2                 = %.6f m\n', 9.81 / sys_est.Parameters(2).Value);
