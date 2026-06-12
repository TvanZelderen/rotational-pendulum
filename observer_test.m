% test_offline_linear_observer.m
% Runs a purely Linear Luenberger Observer using only A, B, C, and L matrices.

clear; clc;
pendulum_params; 

%% 1. Load Data
data_folder = 'data';
file_name   = '20260605_132001_multisine_amp300.mat';
run_data    = load(fullfile(data_folder, file_name));

t_raw = run_data.simin(:,1);
u_raw = run_data.simin(:,2);
y_raw = run_data.simout.Data;
h     = mean(diff(t_raw));
fprintf('Sample time used: h = %.4f s\n', h);

%% 2. Central Difference Linearization (Downward)
fprintf('\n--- Calculating A and B Matrices ---\n');
eps_jac = 1e-6;
x_eq = [0; 0; 0; 0];
A = zeros(4,4);

% Compute A
for i = 1:4
    xp = x_eq; xp(i) = xp(i) + eps_jac;
    xm = x_eq; xm(i) = xm(i) - eps_jac;
    A(:,i) = (rotpen_ode(0, xp, 0, p) - rotpen_ode(0, xm, 0, p)) / (2*eps_jac);
end

% Compute B
u_plus = eps_jac;
u_minus = -eps_jac;
B = (rotpen_ode(0, x_eq, u_plus, p) - rotpen_ode(0, x_eq, u_minus, p)) / (2*eps_jac);

C = [1 0 0 0; 
     0 0 1 0];
D = zeros(2,1);
eig(A)
%% 3. Observer Design (Optimal Kalman / LQR Approach)
fprintf('\n--- Calculating Optimal L Matrix ---\n');

max_th1_err  = deg2rad(1);    % 5 deg angle error acceptable
max_dth1_err = deg2rad(10);   % 50 deg/s velocity error acceptable
max_th2_err  = deg2rad(1);    % 5 deg
max_dth2_err = deg2rad(10);   % 50 deg/s

max_th1_meas_err  = deg2rad(1);   % encoder noise ~ 1 deg
max_th2_meas_err  = deg2rad(1);   % encoder noise ~ 1 deg

% Build Q and R
Q = diag([1/max_th1_err^2, ...
          1/max_dth1_err^2, ...
          1/max_th2_err^2, ...
          1/max_dth2_err^2]);

R = diag([1/max_th1_meas_err^2, ...
          1/max_th2_meas_err^2]);

Q_obs = diag([800, 1500000,800,1500000]);
% Q_obs = diag([1000,5000,1000,5000]);

R_obs = 0.01*diag([1,1]);

% N — cross-covariance between process and measurement noise
% Usually zero
N = zeros(4,2);

%% Build state space model
%% Build state space model
% We build the continuous augmented system just like Simulink
sys_aug = ss(A, [B, eye(4)], C, [D, zeros(2,4)]);

%% Compute CONTINUOUS Kalman gain 
% We use the continuous system so the 1.5 million Q_obs behaves normally!
[~, L_cont, ~] = kalman(sys_aug, Q_obs, R_obs, N);

fprintf('Continuous Kalman gain L:\n'); disp(L_cont)

%% Build the Complete Observer Physics Block
% Instead of doing the A*x + L*y math manually in the loop, 
% we pack the entire (A - L*C) equation into a single State-Space system.
A_obs = A - L_cont * C;
B_obs = [B, L_cont]; % The observer takes 3 inputs: [Motor Command; Sensor 1; Sensor 2]
C_obs = eye(4);      % The observer outputs all 4 estimated states
D_obs = zeros(4, 3);

% Discretize the ENTIRE observer for the offline loop
sys_obs_discrete = c2d(ss(A_obs, B_obs, C_obs, D_obs), h, 'zoh');
[Ad_obs, Bd_obs, ~, ~] = ssdata(sys_obs_discrete);

%% Check observer stability (Discrete poles must be < 1)
ev_obs = eig(Ad_obs);
if all(abs(ev_obs) < 1)
    fprintf('Observer STABLE (All poles inside Unit Circle)\n');
else
    fprintf('WARNING: Observer UNSTABLE\n');
end

%% 4. Data Processing & Trimming
iS = round(3/h) + 1;
iE = min(round(28/h) + 1, length(t_raw));
t_trim = t_raw(iS:iE);
u_trim = u_raw(iS:iE);
N      = length(t_trim);
true_states = zeros(4, N);
true_states(1,:) = deg2rad(y_raw(iS:iE, 1))'; 
true_states(2,:) = deg2rad(y_raw(iS:iE, 2))'; 
true_states(3,:) = deg2rad(y_raw(iS:iE, 3))'; 
true_states(4,:) = deg2rad(y_raw(iS:iE, 4))'; 
fs = 1/h;
fc = 5;                     
[b, a] = butter(2, fc/(fs/2));
dth1_f = filtfilt(b, a, true_states(2,:));
dth2_f = filtfilt(b, a, true_states(4,:));

%% 5. Observer Loop
fprintf('\n--- Running DISCRETE Linear Observer ---\n');
x_hat = zeros(4, N);
x_hat(:,1) = true_states(:,1); % Initialize 

for k = 1:N-1
    if any(isnan(x_hat(:,k))) || any(abs(x_hat(:,k)) > 100)
        fprintf('WARNING: Observer diverged at t=%.2f s\n', t_trim(k));
        x_hat(:,k+1:end) = NaN;
        break;
    end
    
    xk = x_hat(:,k);    
    uk = u_trim(k);     
    y_meas = [true_states(1,k); true_states(3,k)]; 
  
    % The loop is now a perfectly stable, single matrix multiplication!
    inputs = [uk; y_meas];
    x_hat(:,k+1) = Ad_obs * xk + Bd_obs * inputs;
    
end



%% 6. Plotting Results
%% 6. Plotting Results
figure('Name', 'Linear Observer Verification', 'Position', [100, 100, 800, 800]);

% Use tiledlayout to squish the plots together and share the x-axis
t = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

% Use LaTeX interpreter for beautiful Greek letters and dots (derivatives)
labels = {'$\theta_1$ [deg]', '$\dot{\theta}_1$ [deg/s]', ...
          '$\theta_2$ [deg]', '$\dot{\theta}_2$ [deg/s]'};

% Group the correct reference signals to match the 4 states
reference_signals = { rad2deg(true_states(1,:)), ...  % State 1: th1
                      rad2deg(dth1_f), ...            % State 2: filtered dth1
                      rad2deg(true_states(3,:)), ...  % State 3: th2
                      rad2deg(dth2_f) };              % State 4: filtered dth2


nexttile; % Replaces subplot(4,1,i)
    
    % Plot ONE blue reference line and ONE red observer line per subplot
    plot(t_trim, reference_signals{1}, 'b', 'LineWidth', 1.2); hold on;
    plot(t_trim, rad2deg(x_hat(1,:)), 'r--', 'LineWidth', 1.5);
    
    % Apply the LaTeX labels with a slightly larger font for reports
    ylabel(labels{1}, 'Interpreter', 'latex', 'FontSize', 12); 
    grid on;
    xlim([3, 22]);
    lgd = legend('Hardware Data', 'Linear Observer', 'Location', 'best');
    xticklabels({});
nexttile; % Replaces subplot(4,1,i)
    
    % Plot ONE blue reference line and ONE red observer line per subplot
    plot(t_trim, reference_signals{2}, 'b', 'LineWidth', 1.2); hold on;
    plot(t_trim, rad2deg(x_hat(2,:)), 'r--', 'LineWidth', 1.5);
    
    % Apply the LaTeX labels with a slightly larger font for reports
    ylabel(labels{2}, 'Interpreter', 'latex', 'FontSize', 12); 
    grid on;
    xlim([3, 22]);
    ylim([-200,200])
    xticklabels({});
nexttile; % Replaces subplot(4,1,i)
    
    % Plot ONE blue reference line and ONE red observer line per subplot
    plot(t_trim, reference_signals{3}, 'b', 'LineWidth', 1.2); hold on;
    plot(t_trim, rad2deg(x_hat(3,:)), 'r--', 'LineWidth', 1.5);
    
    % Apply the LaTeX labels with a slightly larger font for reports
    ylabel(labels{3}, 'Interpreter', 'latex', 'FontSize', 12); 
    grid on;
    xlim([3, 22]);
    ylim([-100,100])
    xticklabels({});


nexttile;
plot(t_trim, reference_signals{4}, 'b', 'LineWidth', 1.2); hold on;
plot(t_trim, rad2deg(x_hat(4,:)), 'r--', 'LineWidth', 1.5);
    
    % Apply the LaTeX labels with a slightly larger font for reports
    ylabel(labels{4}, 'Interpreter', 'latex', 'FontSize', 12); 
    grid on;
    xlim([3, 22]);
    ylim([-1000,1000])

% Put ONE master X-label at the very bottom of the layout
xlabel(t, 'Time [s]', 'FontSize', 12);
