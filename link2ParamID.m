% Prerequisites (run once per session before this script):
%   1. calib.m  — opens fugiboard connection, resets encoder, activates relay
%   2. hwinit.m — sets sensor gain/offset calibration values
% Re-running calib.m resets the encoder, so only run it when starting fresh.

clear; clc;

%% Parameters
h    = 0.01;  % sample period [s]
Tsim = 30;    % experiment duration [s]

%% Input signal
t = [0:h:Tsim]';
u = 0;  % zero input (open loop), to hold arm 1

% simin is read by the Simulink model: col 1 = time, col 2 = input
simin = [t, u];

%% Run experiment
sim rotpentemplate

%% Extract outputs
% simout is a Timeseries written by the 'To Workspace' block (sample time h)
y = simout.Data;  % N x 2 matrix: [th1, th2] in degrees

%% Wrap angles to (-180, 180] so equilibrium at 0 doesn't jump to ±355
th1 = mod(y(:,1) + 180, 360) - 180;
th2 = mod(y(:,2) + 180, 360) - 180;

%% Plot
plot(t, th1, t, th2)
legend('\theta_1', '\theta_2')
xlabel('Time [s]')
ylabel('Angle [deg]')

%% Trim starting point based on plot
trimStart = 5;
th2Trimmed = th2(trimStart*100 + 1:end);

%% idgrey
data = iddata(th2Trimmed, [], h);          % output-only, no input

% estimates for alpha0
alpha0 = 1;

sys  = idgrey(@link2_ode, alpha0, 'c');
sys_est = greyest(data, sys);