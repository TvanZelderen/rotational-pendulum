% Offline arm-1 ramp-test analysis.
% Collect data first with run_ramp.m.
% Identifies: km (motor torque constant) and tauc_static (Coulomb breakaway torque).

clear; clc;
pendulum_params;   % source of truth for physical constants

%% ── Configuration ───────────────────────────────────────────────────────────
% One file per arm-1 position, saved by run_ramp.m Cell A and Cell B.
file_A      = '20260518_104621_arm1_ramp_th000_a10.mat';         % theta1 = 0 deg,  e.g. '20260514_120000_arm1_ramp_th000_a40.mat'
file_B      = '20260518_104700_arm1_ramp_th090_a10.mat';         % theta1 = 90 deg, e.g. '20260514_120500_arm1_ramp_th090_a40.mat'
data_folder = 'data';

theta1_A_deg = 0;    % [deg] — must match arm position used in Cell A
theta1_B_deg = 90;   % [deg] — must match arm position used in Cell B

%% ── Load data ────────────────────────────────────────────────────────────────
d_A   = load(fullfile(data_folder, file_A));
d_B   = load(fullfile(data_folder, file_B));

t_A    = d_A.simin(:, 1);   u_A = d_A.simin(:, 2);
t_B    = d_B.simin(:, 1);   u_B = d_B.simin(:, 2);

dth1_A = deg2rad(d_A.simout.Data(:, 2));   % [rad/s]
dth1_B = deg2rad(d_B.simout.Data(:, 2));

 % ── Trim data ────────────────────────────────────────────────────────────────
trim_s = 3; % s
trim_n = trim_s * 100 + 1;
t_A    = t_A(1:trim_n);   u_A = u_A(1:trim_n);
t_B    = t_B(1:trim_n);   u_B = u_B(1:trim_n);

dth1_A = dth1_A(1:trim_n);
dth1_B = dth1_B(1:trim_n);

h = mean(diff(t_A));
fprintf('Loaded: %d samples per run at h = %.3f s\n', length(t_A), h);

%% ── Filter ───────────────────────────────────────────────────────────────────
fs = 1/h;
fc = 2;
[b, a] = butter(2, fc/(fs/2));

dth1_A_f = filtfilt(b, a, dth1_A);
dth1_B_f = filtfilt(b, a, dth1_B);

figure('Name', 'Filter check');
subplot(2,1,1);
  plot(t_A, dth1_A, 'Color',[0.8 0.8 0.8]); hold on;
  plot(t_A, dth1_A_f, 'r', 'LineWidth', 1.5);
  ylabel('d\theta_1 [rad/s]'); title('\theta_1 = 0 deg run'); grid on; legend('Raw','Filtered');
subplot(2,1,2);
  plot(t_B, dth1_B, 'Color',[0.8 0.8 0.8]); hold on;
  plot(t_B, dth1_B_f, 'r', 'LineWidth', 1.5);
  ylabel('d\theta_1 [rad/s]'); title('\theta_1 = 90 deg run'); grid on; xlabel('Time [s]');

%% ── Overview plot ────────────────────────────────────────────────────────────
figure('Name', 'Ramp overview');
subplot(2,2,1); plot(t_A, u_A);       ylabel('u [-]');            title('\theta_1 = 0 deg');  grid on;
subplot(2,2,3); plot(t_A, dth1_A_f);  ylabel('d\theta_1 [rad/s]'); xlabel('Time [s]');        grid on;
subplot(2,2,2); plot(t_B, u_B);       title('\theta_1 = 90 deg'); grid on;
subplot(2,2,4); plot(t_B, dth1_B_f);  xlabel('Time [s]');         grid on;

%% ── Fit: km and tauc_static ──────────────────────────────────────────────────
% The ramp is run at two arm-1 positions so that the gravity torque gives a
% known reference to separate km from tauc_static.
%
%   km * u_break = tauc_static + m1 * g * lc1 * sin(theta1_rad)


u_break_A = 0.097;
u_break_B = 0.105;

A = [u_break_A, -1; u_break_B, -1];
B = [0; (p.m1*p.lc1 + p.m2*p.l1)*p.g*sin(deg2rad(90))];
x = A\B;

km = x(1,:);
tauc_static = x(2,:);


%% ── Results ─────────────────────────────────────────────────────────────────
fprintf('\n--- Arm-1 ramp identification ---\n');
fprintf('  km          = %.6f  N·m/unit\n', km);
fprintf('  tauc_static = %.6f  N·m\n', tauc_static);
