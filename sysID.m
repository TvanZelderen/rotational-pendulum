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
y = simout.Data;  % N x 2 matrix: [th1, th2] in degrees

%% Wrap angles to (-180, 180] so equilibrium at 0 doesn't jump to ±355
th1 = mod(y(:,1) + 180, 360) - 180;
th2 = mod(y(:,2) + 180, 360) - 180;

%% Trim starting point based on plot
trimStart = 0;
th2Trimmed = th2(trimStart*100 + 1:end);



% 1. Create model structure (no symbols used here!)
model = idgrey('my_pendulum_model', [0.003, 0, 0, 0, 0, 0], 'c', [m1_val, l1_val, g_val]);

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



% 1. Create model structure (no symbols used here!)
model = idgrey('my_pendulum_model', [0.003, 0, 0, 0, 0, 0], 'c', [m1_val, l1_val, g_val]);

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

function [A, B, C, D] = my_pendulum_model(I1, p1, p2, p3, p4, b, T, aux)
    % aux contains [m1, l1, g] which you measured in the lab
    m1 = aux(1); l1 = aux(2); g = aux(3);
    % Use the formulas derived in your White Box step
    a = (m1 * g * l1) / I1; % Linearized term from Slide 10
    
    A = [0,  1,  0,  0;
         a,  0,  p1, 0; % p1 is the 'coupling' unknown
         0,  0,  0,  1;
         p2, 0,  p3, p4];
         
    B = [0; b; 0; 0]; % Direct motor effect on Link 1
    C = eye(4); D = 0;
end
