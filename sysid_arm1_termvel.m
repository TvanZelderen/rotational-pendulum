% Offline arm-1 terminal-velocity sweep analysis.
% Collect data first with run_arm1_terminal_vel.m.
% Identifies: kbc1 (composite back-EMF + joint damping) and tauc_kinetic
% (Coulomb kinetic friction torque).

clear; clc;
pendulum_params;   % source of truth; km must be set from sysid_arm1_ramp first

%% ── Configuration ───────────────────────────────────────────────────────────
file_name    = '';         % e.g. '20260514_130000_arm1_termvel.mat'
data_folder  = 'data';

trim_start   = 2;          % [s] skip initial transient
trim_end     = 60;         % [s] adjust to match sweep duration

% Settling time — how many seconds at each step level to treat as steady-state?
% Use the last settle_frac fraction of each constant-u segment.
settle_frac  = 0.4;        % last 40% of each step segment → steady-state window

%% ── Load data ────────────────────────────────────────────────────────────────
run_data = load(fullfile(data_folder, file_name));
t_raw  = run_data.simin(:, 1);
u_raw  = run_data.simin(:, 2);
y_raw  = run_data.simout.Data;   % [N×5]: [th1_deg, dth1_dps, th2_deg, dth2_dps, psi_deg]
h      = mean(diff(t_raw));

th1_deg  = y_raw(:, 1);
dth1_dps = y_raw(:, 2);
th1_rad  = deg2rad(th1_deg);
dth1_rad = deg2rad(dth1_dps);

%% ── Trim ─────────────────────────────────────────────────────────────────────
i_start = round(trim_start / h) + 1;
i_end   = min(round(trim_end / h) + 1, length(t_raw));

t_trimmed    = t_raw(i_start:i_end);
u_trimmed    = u_raw(i_start:i_end);
th1_trimmed  = th1_rad(i_start:i_end);
dth1_trimmed = dth1_rad(i_start:i_end);

fprintf('Trim window: t = %.2f to %.2f s  (%d samples)\n', ...
        t_trimmed(1), t_trimmed(end), length(t_trimmed));

%% ── Filter ───────────────────────────────────────────────────────────────────
fs = 1/h;
fc = 5;
[b, a] = butter(2, fc/(fs/2));

dth1_f = filtfilt(b, a, dth1_trimmed);
u_f    = filtfilt(b, a, u_trimmed);   % smooth u for step-edge detection

figure('Name', 'Filter check — dth1');
plot(t_trimmed, dth1_trimmed, 'Color', [0.8 0.8 0.8]); hold on;
plot(t_trimmed, dth1_f, 'r', 'LineWidth', 1.5);
ylabel('Angular velocity [rad/s]'); xlabel('Time [s]');
legend('Raw', 'Filtered'); title('Filter verification'); grid on;

%% ── Overview plot ────────────────────────────────────────────────────────────
figure('Name', 'Terminal-velocity overview');
subplot(2,1,1); plot(t_trimmed, u_trimmed); ylabel('u [-]'); grid on; title('Input');
subplot(2,1,2); plot(t_trimmed, dth1_f);    ylabel('d\theta_1 [rad/s]'); xlabel('Time [s]'); grid on;

%% ── Step segmentation ────────────────────────────────────────────────────────
% TODO: detect the edges between constant-u steps and segment the signal.
%
% Hint: where du/dt is large, a step transition is occurring.
%       Between transitions, u is (approximately) constant.
%
% For each segment, check: does arm 1 reach a steady angular velocity?
% Discard segments that are too short or that end in a transition.

% Placeholder outputs — replace with your segmentation results:
u_levels  = [];   % one value per step [normalised], Nx1
omega_ss  = [];   % corresponding steady-state dth1 [rad/s], Nx1

%% ── Fit: kbc1 and tauc_kinetic ───────────────────────────────────────────────
% At terminal velocity, angular acceleration = 0, so the EOM for arm 1
% (A2 — stiff arm) simplifies to:
%
%   km * u = kbc1 * omega_ss + tauc_kinetic * sign(omega_ss)
%
% You have km from sysid_arm1_ramp.m and a set of (u, omega_ss) pairs.
%
% TODO:
%   1. Cast the equation above as a linear regression (Ax = b).
%      Identify what your regression variables and regressors are.
%   2. Fit using \  (backslash).
%   3. Check: is R² > 0.9? If not, what could explain the scatter?
%   4. Do you get different kbc1/tauc_kinetic for positive vs negative u?
%      (Asymmetric Coulomb friction is common — investigate if residuals suggest it.)

kbc1         = NaN;   % [N·m·s/rad]
tauc_kinetic = NaN;   % [N·m]

%% ── Plots ────────────────────────────────────────────────────────────────────
% TODO: plot (u_levels, omega_ss) with the fitted line overlaid.
%       Add a residuals subplot to check scatter.

%% ── Results ─────────────────────────────────────────────────────────────────
fprintf('\n--- Arm-1 terminal-velocity identification ---\n');
fprintf('  kbc1         = %.6f  N·m·s/rad\n', kbc1);
fprintf('  tauc_kinetic = %.6f  N·m\n', tauc_kinetic);
fprintf('\nNext: update pendulum_params.m with km, kbc1, tauc_kinetic,\n');
fprintf('then validate with sysid_arm1_driven.m.\n');
