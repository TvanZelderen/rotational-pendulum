%SYSTEM IDENTIFICATION

% Define your lab measurements
m1_val = 0.1;     % kg
l1_val = 0.1;    % meters (center of mass)
g_val  = 9.81;    % m/s^2

% Prerequisites (run once per session before this script):
%   1. calib.m  — opens fugiboard connection, resets encoder, activates relay
%   2. hwinit.m — sets sensor gain/offset calibration values
% Re-running calib.m resets the encoder, so only run it when starting fresh.


%% Parameters
h    = 0.01;  % sample period [s]
Tsim = 30;    % experiment duration [s]

%% Input signal
t = [0:h:Tsim]';

amplitude = 0.5;
omega     = 5;
u = amplitude * sin(omega * t); 

% simin is read by the Simulink model: col 1 = time, col 2 = input
simin = [t, u];

%% Run experiment
sim rotpentemplate

%% Extract outputs
% simout is a Timeseries written by the 'To Workspace' block (sample time h)
y = simout.Data;  % N x 4 matrix: [th1, dth1, th2, dth2] in degrees

%% Wrap angles to (-180, 180] so equilibrium at 0 doesn't jump to ±355
th1 = mod(y(:,1) + 180, 360) - 180;
dth1 = y(:,2);
th2 = mod(y(:,3) + 180, 360) - 180;
dth1 = y(:,4);

%% Trim starting point based on plot
trimStart = 0;
th2Trimmed = th2(trimStart*100 + 1:end);



% 1. Create model structure (no symbols used here!)
model = idgrey(@my_pendulum_model, [0.003, 0, 0, 0, 0, 0], 'c', [m1_val, l1_val, g_val]);

% y_lab = [theta1, dtheta1, theta2, dtheta2]; 
% u_lab = [torque];
%lab_data_object = iddata(y_lab, u_lab, Ts);

data = iddata(th2Trimmed, u, h); 

% 2. Identify the unknowns using your lab data
estimated_model = greyest(data, model); 

% 3. Extract the final NUMERIC matrices
% These are now arrays of pure numbers (doubles). No symbols!
A_final = estimated_model.A; 
B_final = estimated_model.B;

%Error using idgrey (line 367)
%The ODE function "my_pendulum_model" could not be evaluated successfully using
%the given set of parameters, sample time and optional arguments. The error
%message generated during the evaluation was:
%Not enough input arguments.

%Error in sysID (line 46)
%model = idgrey(@my_pendulum_model, [0.003, 0, 0, 0, 0, 0], 'c', [m1_val, l1_val, g_val]);