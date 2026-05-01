%% Sample code to run experiment from script,
% specifying a fixed input signal and recording the measured
% outputs to a variable in the Workspace for further processing.

clear;clc;

% Sample rate in sec.
h = 0.01;

% Experiment duration in sec.
% (don't forget to change this in your diagram, see video)
Tsim = 30;

% Time vector (don't forget to transpose with ')
t = [0:h:Tsim]';

% Input vector
amplitude = 0.5;
omega = 0.1;
u = amplitude * sin(omega * t) * 0;

% Variable that goes to Simulink
% (First column: time, Second column: input values)
simin = [t, u]

%% Start experiment
sim rotpentemplate

%% Collect output data
% (make sure that samples are taken every 'h' seconds! in 'To Workspace' block)

% If output type 'Timeseries'
y = simout.Data;

% If output type 'Array'
% y = simout;

%% Plot data
th1 = y(:, 1);
th1 = mod(th1, 360);
th2 = y(:, 2);
th2 = mod(th2, 360);

theta1_mean = mean(th1(100:end-100))
theta2_mean = mean(th2(100:end-100))
theta1_range = max(th1(100:end-100)) - min(th1(100:end-100))
theta2_range = max(th2(100:end-100)) - min(th2(100:end-100))

plot(t, th1, t, th2)
legend('theta_1', 'theta_2')
