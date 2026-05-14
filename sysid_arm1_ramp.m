% Offline arm-1 ramp-test analysis.
% Collect data first with run_arm1_ramp.m.
% Identifies: km (motor torque constant) and tauc_static (Coulomb breakaway torque).

clear; clc;
pendulum_params;   % source of truth for physical constants

%% ── Configuration ───────────────────────────────────────────────────────────
file_name   = '';         % e.g. '20260514_120000_arm1_ramp.mat'
data_folder = 'data';

trim_start  = 2;          % [s] skip initial transient
trim_end    = 30;         % [s] adjust to match your ramp duration

% Angles at which the ramp test was run (arm 1 held at each before ramping)
theta1_test_deg = [0, 90];   % [deg] — must match what run_arm1_ramp.m used

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
fc = 5;                              % cutoff [Hz] — lower than driven ID since signal is slow
[b, a] = butter(2, fc/(fs/2));

th1_f  = filtfilt(b, a, th1_trimmed);
dth1_f = filtfilt(b, a, dth1_trimmed);

figure('Name', 'Filter check — dth1');
plot(t_trimmed, dth1_trimmed, 'Color', [0.8 0.8 0.8]); hold on;
plot(t_trimmed, dth1_f, 'r', 'LineWidth', 1.5);
ylabel('Angular velocity [rad/s]'); xlabel('Time [s]');
legend('Raw', 'Filtered'); title('Filter verification'); grid on;

%% ── Overview plot ────────────────────────────────────────────────────────────
figure('Name', 'Ramp overview');
subplot(3,1,1); plot(t_trimmed, rad2deg(th1_trimmed)); ylabel('\theta_1 [deg]'); grid on;
subplot(3,1,2); plot(t_trimmed, u_trimmed); ylabel('u [-]'); grid on;
subplot(3,1,3); plot(t_trimmed, dth1_f); ylabel('d\theta_1 [rad/s]'); xlabel('Time [s]'); grid on;

%% ── Fit: km and tauc_static ──────────────────────────────────────────────────
% The ramp is run at two arm-1 positions so that the gravity torque gives a
% known reference to separate km from tauc_static.
%
% At each test angle theta1, the motor just overcomes static friction + gravity:
%
%   km * u_b = tauc_static + m1 * g * lc1 * sin(theta1)
%
% where u_b is the breakaway input (the u at which arm 1 first moves).
%
% You measure u_b at two angles → two equations, two unknowns (km, tauc_static).
%
% TODO:
%   1. For each test angle, isolate the corresponding segment of the ramp.
%   2. Find the breakaway instant — how does dth1_f behave before vs. after?
%      What threshold makes sense given your noise floor?
%   3. Extract u_b at each breakaway.
%   4. Write and solve the 2×2 linear system.

km          = NaN;   % [N·m per normalised unit]
tauc_static = NaN;   % [N·m]

%% ── Plots ────────────────────────────────────────────────────────────────────
% TODO: add breakaway markers to the overview plot so the fit is visually verifiable.

%% ── Results ─────────────────────────────────────────────────────────────────
fprintf('\n--- Arm-1 ramp identification ---\n');
fprintf('  km          = %.6f  N·m/unit\n', km);
fprintf('  tauc_static = %.6f  N·m\n', tauc_static);
fprintf('\nNext: run sysid_arm1_termvel.m to identify kbc1 and tauc_kinetic.\n');
fprintf('Then update pendulum_params.m and validate with sysid_arm1_driven.m.\n');
