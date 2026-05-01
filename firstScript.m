clear; clc;

%% Parameters
h    = 0.01;  % sample period [s]
Tsim = 30;    % experiment duration [s]

%% Input signal
t = [0:h:Tsim]';

amplitude = 0.5;
omega     = 0.1;
u = amplitude * sin(omega * t) * 0;  % zero input (open loop)

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

%% Steady-state statistics (trim first and last second)
th1_ss = th1(101:end-100);
th2_ss = th2(101:end-100);

theta1_mean  = mean(th1_ss)
theta2_mean  = mean(th2_ss)
theta1_range = max(th1_ss) - min(th1_ss)
theta2_range = max(th2_ss) - min(th2_ss)

%% Plot
plot(t, th1, t, th2)
legend('\theta_1', '\theta_2')
xlabel('Time [s]')
ylabel('Angle [deg]')
