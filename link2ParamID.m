clear; clc;
% Prerequisites (run once per session before this script):
%   1. calib.m  — opens fugiboard connection, resets encoder, activates relay
%   2. hwinit.m — sets sensor gain/offset calibration values
% Re-running calib.m resets the encoder, so only run it when starting fresh.

%% Parameters
h    = 0.01;  % sample period [s]
Tsim = 30;    % experiment duration [s]

%% Input signal
t = [0:h:Tsim]';
u = zeros(size(t));  % zero input (open loop), to hold arm 1

% simin is read by the Simulink model: col 1 = time, col 2 = input
simin = [t, u];

%% Run experiment
sim rotpentemplate

%% Extract outputs
% simout is a Timeseries written by the 'To Workspace' block (sample time h)
y = simout.Data;  % N x 4 matrix: [th1, dth1, th2, dth2] in degrees

%% Wrap angles to (-180, 180] so equilibrium at 0 doesn't jump to ±355
th1 = mod(y(:,1) + 180, 360) - 180;
th2 = mod(y(:,3) + 180, 360) - 180;

%% Plot
plot(t, th1, t, th2)
legend('\theta_1', '\theta_2')
xlabel('Time [s]')
ylabel('Angle [deg]')

%% Trim starting point based on plot
trimStart = 1.5;
trimEnd = 30;
tTrimmed = t(trimStart*100+1:end-(Tsim-trimEnd)*100);
th1Trimmed = th1(trimStart*100 + 1:end-(Tsim-trimEnd)*100);
theta1Mean = mean(th1Trimmed);
th2Trimmed = th2(trimStart*100 + 1:end-(Tsim-trimEnd)*100);
th2Zeroed = th2Trimmed + theta1Mean;
th2Modded = mod(th2Zeroed + 180, 360) - 180;

%% Plot
plot(tTrimmed, th2Modded)
legend('\theta_2')
xlabel('Time [s]')
ylabel('Angle [deg]')

%% Convert to radians (sin() in the nonlinear ODE requires radians)
th2_rad = deg2rad(th2Modded);

%% idnlgrey — nonlinear grey-box identification
data = iddata(th2_rad, [], h);             % output-only, no input

% Initial parameter guesses
alpha0 = 1;
beta0  = 9.81 / 0.1;

% Initial state guess: start from first measured angle, zero velocity
x0 = [th2_rad(1); 0];

% idnlgrey(FileName, [ny nu nx], Parameters, InitialStates, Ts)
sys = idnlgrey('link2_ode', [1, 0, 2], {alpha0; beta0}, x0, 0);

% Allow the initial state to be estimated along with the parameters
sys.InitialStates(1).Fixed = false;
sys.InitialStates(2).Fixed = false;

opt = nlgreyestOptions('Display', 'on');
sys_est = nlgreyest(data, sys, opt);

fprintf('alpha (c2/m2/l2^2) = %.6f\n', sys_est.Parameters(1).Value);
fprintf('beta  (g/l2)       = %.6f\n', sys_est.Parameters(2).Value);
fprintf('l_2 = %.6f\n', sys_est.Parameters(2).Value/9.81);