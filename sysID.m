%SYSTEM IDENTIFICATION
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
% Order: [4 outputs, 1 input, 4 states]
order = [4, 1, 4]; 

% --- MEASURED KNOWNS (aux) ---
% aux = [m1, L1, m2, L2, g]
% Use your actual measurements here!
g_val  = 9.81;    % m/s^2
L1_val = 0.1; % Example: 10cm total length
L2_val = 0.1; 
m2_val = 0.05; 
m1_val = 0.1;

% 1. Create the big parameter list (12 values)
% [I1, I2, kt, c1, c2, lc1, lc2, m1, L1, m2, L2, g]
p_init = [0.003; 0.001; 10; 0.01; 0.01; 0.05; 0.05; m1_val; L1_val; m2_val; L2_val; 9.81];

% 2. Create the model (Notice: NO FileArgument needed!)
% Order: [4 outputs, 1 input, 12 parameters]
x0 = [th1_trimmed(1); dth1_trimmed(1); th2_trimmed(1); dth2_trimmed(1)];
nl_model = idnlgrey(@double_pen, [4 1 4], p_init, x0, 0);

nl_model.SimulationOptions.AbsTol = 1e-6;
nl_model.SimulationOptions.RelTol = 1e-5;
% 3. LOCK THE KNOWNS (Crucial Step)
% We tell MATLAB: "Do not touch parameters 8, 9, 10, 11, and 12"
for k = 8:12
    nl_model.Parameters(k).Fixed = true;
end

% 4. Set Bounds for the ones it CAN change
nl_model.Parameters(6).Maximum = L1_val; % lc1
nl_model.Parameters(7).Maximum = L2_val; % lc2
for k = 1:5
    nl_model.Parameters(k).Minimum = 0; % I, kt, c must be positive
end

% 5. Run Estimation
opt = nlgreyestOptions('Display', 'on', 'SearchMethod', 'lm');
estimated_model = nlgreyest(data, nl_model, opt);



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
