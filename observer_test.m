% test_offline_linear_observer.m
% Runs a purely Linear Luenberger Observer using only A, B, C, and L matrices.

clear; clc;
pendulum_params; 

%% 1. Load Data
data_folder = 'data';
file_name   = '20260527_125403_multisine_amp400.mat';
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

%% 3. Observer Design (Optimal Kalman / LQR Approach)
fprintf('\n--- Calculating Optimal L Matrix ---\n');

% Q relates to how much we want the observer to aggressively track the states.
% (Higher numbers = track states faster)
Q_obs = diag([1000, 0, 1000, 0]); 

% R relates to the encoders (we thrust them fully)
R_obs = diag([1, 1]); 

% Calculate L using the Linear Quadratic Estimator (Dual LQR) method.
L = lqr(A', C', Q_obs, R_obs);

fprintf('Max |L| gain = %.2f\n', max(abs(L(:))));

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
fc = 5;                     % Cutoff frequency (Hz)
[b, a] = butter(2, fc/(fs/2));

dth1_f = filtfilt(b, a, true_states(2,:));

dth2_f = filtfilt(b, a, true_states(4,:));
%% 5.Observer Loop
fprintf('\n--- Running Linear Observer ---\n');
x_hat = zeros(4, N);
x_hat(:,1) = true_states(:,1); % Initialize 


for k = 1:N-1
    if any(isnan(x_hat(:,k))) || any(abs(x_hat(:,k)) > 100)
        fprintf('WARNING: Observer diverged at t=%.2f s\n', t_trim(k));
        x_hat(:,k+1:end) = NaN;
        break;
    end
    
    % Get the current real-world sensor data
    y_meas = [true_states(1,k); true_states(3,k)]; 
  
    xk = x_hat(:,k);    % Grab current observer state
    uk = u_trim(k);     % Grab current motor comman
    
    % Calculate how many tiny math steps fit inside one hardware step
    math_step_size = 0.0001;
    sub_steps = max(1, round(h / math_step_size)); 
    h_math = h / sub_steps; % Guarantee perfect time alignment
    
    % Run the physics engine 'sub_steps' times before looking at the next sensor data
    for s = 1:sub_steps
        error_y = y_meas - C * xk;
        dx_hat  = A * xk + B * uk + L * error_y;
        xk      = xk + (h_math * dx_hat);
    end
    
    % Save the state at the exact moment it catches up to the hardware time
    x_hat(:,k+1) = xk;
end




%% 6. Plotting Results
figure('Name', 'Linear Observer Verification', 'Position', [100, 100, 800, 800]);
labels = {'th1 [deg]', 'dth1 [deg/s]', 'th2 [deg]', 'dth2 [deg/s]'};

% Group the correct reference signals to match the 4 states
reference_signals = { rad2deg(true_states(1,:)), ...  % State 1: th1
                      rad2deg(dth1_f), ...            % State 2: filtered dth1
                      rad2deg(true_states(3,:)), ...  % State 3: th2
                      rad2deg(dth2_f) };              % State 4: filtered dth2

for i = 1:4
    subplot(4,1,i);
    % Plot ONE blue reference line and ONE red observer line per subplot
    plot(t_trim, reference_signals{i}, 'b', 'LineWidth', 1.2); hold on;
    plot(t_trim, rad2deg(x_hat(i,:)), 'r--', 'LineWidth', 1.5);
    
    ylabel(labels{i}, 'Interpreter', 'none'); 
    grid on;
    
    if i == 1
        legend('Hardware Data (Filtered)', 'Linear Observer', 'Location', 'best');
        title('Linear Luenberger Observer Tracking (Matrix Math Only)');
    end
end
xlabel('Time [s]');
