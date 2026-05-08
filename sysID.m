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

%% Input signald
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
dth2 = y(:,4);

%% Trim starting point based on plot
trimStart = 0;

th1_rad = th1 * (pi/180);
dth1_rad = dth1 * (pi/180);
th2_rad = th2 * (pi/180);
dth2_rad = dth2 * (pi/180);

% Trim both to match lengths
uTrimmed = u(trimStart*100 + 1:end);

th1_trimmed = th1_rad(trimStart*100 + 1 : end);
dth1_trimmed = dth1_rad(trimStart*100 + 1 : end);
th2_trimmed = th2_rad(trimStart*100 + 1 : end);
dth2_trimmed = dth2_rad(trimStart*100 + 1 : end);



data = iddata([th1_trimmed, dth1_trimmed, th2_trimmed, dth2_trimmed], uTrimmed, h);

% 2. Setup the Non-linear Model
order = [4, 1, 4]; 
initial_pars = {0.001, 0, 0, 0, 0, 1.0}; % Cell array is correct
aux_data = [m1_val, l1_val, g_val];

nl_model = idnlgrey(@pendulum_non_linear, order, initial_pars, [0;0;0;0]);

% CRITICAL: Use FileArgument so it shows up in varargin{1}
nl_model.FileArgument = aux_data; 

%% 3. Run the Estimation
opt = nlgreyestOptions;
opt.Display = 'on';
opt.SearchMethod = 'gn'; 

% Safety for unstable systems (the inverted pendulum)
opt.Focus = 'simulation'; 

estimated_nl_model = nlgreyest(data, nl_model, opt);








% Create the iddata object

% 1. Create model structure (no symbols used here!)
%model = idgrey(@my_pendulum_model, [0.003, 0, 0, 0, 0, 1], 'c', [m1_val, l1_val, g_val]);

% y_lab = [theta1, dtheta1, theta2, dtheta2]; 
% u_lab = [torque];

% Create the iddata object
%data = iddata([th1_trimmed, dth1_trimmed, th2_trimmed, dth2_trimmed], uTrimmed, h);

%opt = greyestOptions;
%opt.InitialState = 'zero';        % Or 'estimate' if the pendulum wasn't perfectly still
%opt.Focus = 'simulation';         % CRITICAL: Tells MATLAB to look at the overall trajectory
%opt.EnforceStability = false;     % Necessary because your A matrix IS unstable

% 2. Run the identification with these options
%estimated_model = greyest(data, model, opt);

% 3. Extract the final NUMERIC matrices5
% These are now arrays of pure numbers (doubles). No symbols!
%A_final = estimated_model.A; 
%B_final = estimated_model.B;
