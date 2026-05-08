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

%% Extract and wrap angles to (-180, 180]
y   = simout.Data;           % N x 5: [th1, dth1, th2, dth2, phi] in degrees
th1 = mod(y(:,1) + 180, 360) - 180;
th2 = mod(y(:,3) + 180, 360) - 180;
phi = y(:,5);

%% Overview plot — use to set trimStart / trimEnd
figure;
plot(t, th1, t, th2)
legend('\theta_1', '\theta_2')
xlabel('Time [s]'); ylabel('Angle [deg]')

plot(t, phi)
xlabel('Time [s]'); ylabel('Angle [deg]')

%% Trim start and end
trimStart = 1.5;   % [s] — skip initial impulse
trimEnd   = 30;    % [s] — cut settled tail

iStart = trimStart * 100 + 1;
iEnd   = numel(t) - (Tsim - trimEnd) * 100;

th1_trimmed = th1(iStart:iEnd);
th2_trimmed = th2(iStart:iEnd);
tTrimmed    = t(iStart:iEnd);

%% Re-centre th2 around 0 and re-wrap
th2_trimmed = mod(th2_trimmed + mean(th1_trimmed) + 180, 360) - 180;

%% Trimmed data plot
figure;
plot(tTrimmed, th2_trimmed)
legend('\theta_2')
xlabel('Time [s]'); ylabel('Angle [deg]')

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
