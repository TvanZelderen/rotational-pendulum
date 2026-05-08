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

%% Trim: remove impulse at start and settled region at end
trimStart    = 1.5;          % [s]  — skip initial impulse
activeThresh = 2;            % [deg] — cut once pendulum has settled below this

iStart       = round(trimStart / h) + 1;
th2_trimmed  = th2(iStart:end);
iEnd         = find(abs(th2_trimmed) > activeThresh, 1, 'last');
th2_trimmed  = th2_trimmed(1:iEnd);
th1_trimmed  = th1(iStart : iStart + iEnd - 1);
tTrimmed     = t(iStart : iStart + iEnd - 1);

%% Re-centre th2: equilibrium shifts with link 1 position
th2_trimmed  = th2_trimmed - mean(th1_trimmed);

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
