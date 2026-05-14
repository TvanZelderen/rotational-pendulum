clear; clc;
pendulum_params;   % populates struct p — source of truth for guesses + fixed values

%% ── Configuration ──────────────────────────────────────────────────────────
file_name   = '20260513_163112_arm1_pre_id_f200mHz_a30.mat';
data_folder = 'data';

trim_start = 3;     % [s] skip initial transient
trim_end   = 28;    % [s]

%% ── Load data ───────────────────────────────────────────────────────────────
run_data = load(fullfile(data_folder, file_name));
t_raw  = run_data.simin(:, 1);
u_raw  = run_data.simin(:, 2);
y_raw  = run_data.simout.Data;   % [N×5]: [th1_deg, dth1_dps, th2_deg, dth2_dps, psi_deg]
h      = mean(diff(t_raw));

th1_rad  = deg2rad(y_raw(:, 1));
dth1_rad = deg2rad(y_raw(:, 2));
th2_rad  = deg2rad(y_raw(:, 3));
dth2_rad = deg2rad(y_raw(:, 4));

%% ── Trim: calmest start within wrap-safe window ─────────────────────────────
i_start = round(trim_start / h) + 1;
i_end   = min(round(trim_end / h) + 1, length(t_raw));

wrap_mask  = abs(th1_rad(i_start:i_end)) < deg2rad(170) & ...
             abs(th2_rad(i_start:i_end)) < deg2rad(170);

if sum(wrap_mask) < 500
    warning('sysid_arm1_driven: fewer than 5 s of wrap-safe data — widen trim window.');
end

v_combined = abs(dth1_rad(i_start:i_end)) + abs(dth2_rad(i_start:i_end));
v_combined(~wrap_mask) = inf;
[~, i_min] = min(v_combined);
i_start    = i_start + i_min - 1;

y_trimmed = [th1_rad(i_start:i_end),  dth1_rad(i_start:i_end), ...
             th2_rad(i_start:i_end),  dth2_rad(i_start:i_end)];
u_trimmed  = u_raw(i_start:i_end);
t_trimmed  = t_raw(i_start:i_end);

fprintf('Trim window: t = %.2f to %.2f s  (%d samples)\n', ...
        t_trimmed(1), t_trimmed(end), length(t_trimmed));

% plot(t_trimmed, y_trimmed(:,1))

% figure; plot(t_trimmed, u_trimmed/max(abs(u_trimmed)), 'b', t_trimmed, th1_rad(i_start:i_end)/max(abs(th1_rad(i_start:i_end))), 'r');
% legend('u (normalised)', 'th1 (normalised)'); grid on;
% title('Sign check — do they go the same direction?');

%% 1. Design the Filter
fs = 1/h;                    % Your sample rate (100 Hz)
fc = 5;                     % Cutoff frequency (20 Hz)
[b, a] = butter(2, fc/(fs/2)); % 2nd order Butterworth

%% 2. Apply Zero-Phase Filtering (CRITICAL)
% We use 'filtfilt' instead of 'filter' to ensure there is 0ms time delay.
% Standard filters shift the data in time, which ruins inertia estimation!
th1_f  = filtfilt(b, a, th1_rad);
dth1_f = filtfilt(b, a, dth1_rad);
th2_f  = filtfilt(b, a, th2_rad);
dth2_f = filtfilt(b, a, dth2_rad);

%% 3. Diagnostic Check (Don't skip this!)
% Make sure the red line (filtered) goes through the middle of the gray (raw)
figure(10); clf;
% Change this line:
plot(t_trimmed, dth1_rad(i_start:i_end), 'Color', [0.8 0.8 0.8]); hold on;
plot(t_trimmed, dth1_f(i_start:i_end), 'r', 'LineWidth', 1.5);
ylabel('Velocity [rad/s]');
legend('Raw (Fuzzy)', 'Filtered (Clean)');
title('Filter Verification');

%% 4. Create the Clean iddata
y_clean = [th1_f(i_start:i_end), th2_f(i_start:i_end)];
data = iddata(y_clean, u_trimmed, h);
%% ── iddata ──────────────────────────────────────────────────────────────────
%y_angles = [th1_rad(i_start:i_end), th2_rad(i_start:i_end)];
%data = iddata(y_angles, u_trimmed, h);
data.OutputName = {'th1', 'th2'};
data.OutputUnit = {'rad', 'rad'};
data.InputName  = {'u'};
data.InputUnit  = {'normalised'};
data.Tstart     = t_trimmed(1);

%% ── ODE diagnostic ──────────────────────────────────────────────────────────
% Call rotpen_ode_idnlgrey directly with the current p values.
% Verifies no NaN/Inf before handing to the optimiser.
x0_diag = y_trimmed(1, :)';
u0_diag = u_trimmed(1);

[dx_diag, ~] = rotpen_ode_idnlgrey(0, x0_diag, u0_diag, ...
    p.km, p.kbc1, p.c2, p.J1, p.J2, p.l1, p.l2, p.lc1, p.m1, p.m2, p.g);

fprintf('\nODE diagnostic (initial params, t=0):\n');
fprintf('  dth1   = %+.4f rad/s\n',  dx_diag(1));
fprintf('  ddth1  = %+.4f rad/s²\n', dx_diag(2));
fprintf('  dth2   = %+.4f rad/s\n',  dx_diag(3));
fprintf('  ddth2  = %+.4f rad/s²\n', dx_diag(4));
if any(isnan(dx_diag)) || any(isinf(dx_diag))
    error('ODE returns NaN/Inf — check initial parameters and trim window.');
end

%% ── Quick ode45 test ────────────────────────────────────────────────────────
ode_fun  = @(t_ode, x_ode) rotpen_ode_idnlgrey(t_ode, x_ode, ...
    interp1(t_trimmed, u_trimmed, t_ode, 'linear', 'extrap'), ...
    p.km, p.kbc1, p.c2, p.J1, p.J2, p.l1, p.l2, p.lc1, p.m1, p.m2, p.g);
ode_opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
period_s = 1 / 0.2;                              % input period [s] — update if freq changes
n_test   = min(round(2 * period_s / h), length(t_trimmed));   % 2 full periods

[t_test, x_test] = ode45(ode_fun, t_trimmed(1:n_test), x0_diag, ode_opts);

ode_fun_gravity = @(t_ode, x_ode) rotpen_ode_idnlgrey(t_ode, x_ode, 0, ...
    p.km, p.kbc1, p.c2, p.J1, p.J2, p.l1, p.l2, p.lc1, p.m1, p.m2, p.g);
[t_grav, x_grav] = ode45(ode_fun_gravity, t_trimmed(1:n_test), x0_diag, ode_opts);

u_test = interp1(t_trimmed, u_trimmed, t_test, 'linear', 'extrap');

figure('Name', 'ODE test — check for blow-up (initial params)');
subplot(3,1,1);
  plot(t_test, rad2deg(x_test(:,1)), 'b',  t_test, rad2deg(x_test(:,3)),  'r'); hold on;
  plot(t_grav, rad2deg(x_grav(:,1)), 'b--', t_grav, rad2deg(x_grav(:,3)), 'r--');
  legend('\theta_1', '\theta_2', '\theta_1 (gravity only)', '\theta_2 (gravity only)');
  ylabel('Angle [deg]'); title('ODE test — initial param guess'); grid on;
subplot(3,1,2);
  plot(t_test, x_test(:,2), 'b', t_test, x_test(:,4), 'r'); hold on;
  plot(t_grav, x_grav(:,2), 'b--', t_grav, x_grav(:,4), 'r--');
  legend('d\theta_1', 'd\theta_2', 'd\theta_1 (grav)', 'd\theta_2 (grav)');
  ylabel('[rad/s]'); grid on;
subplot(3,1,3);
  plot(t_test, u_test, 'k');
  ylabel('u [-]'); xlabel('Time [s]'); grid on;

%% ── Build idnlgrey model ─────────────────────────────────────────────────────
% Parameter order: km, kbc1, c2, J1, J2, l1, l2, lc1, m1, m2, g
% (must match rotpen_ode_idnlgrey.m argument list)
param_cell = {p.km; p.kbc1; p.c2; p.J1; p.J2; p.l1; p.l2; p.lc1; p.m1; p.m2; p.g};
x0_est     = y_trimmed(1, :)';

sys0 = idnlgrey('rotpen_ode_idnlgrey', [2 1 4], param_cell, x0_est, 0);

param_names = {'km','kbc1','c2','J1','J2','l1','l2','lc1','m1','m2','g'};
free_params = {'km', 'kbc1'};   % everything else fixed

for i = 1:numel(param_names)
    sys0.Parameters(i).Name    = param_names{i};
    sys0.Parameters(i).Minimum = 0;
    sys0.Parameters(i).Fixed   = ~ismember(param_names{i}, free_params);
end
sys0.Parameters(1).Maximum = 30;   % km   [N·m per normalised unit]
sys0.Parameters(2).Maximum = 10;   % kbc1 [N·m·s/rad]

for i = 1:4
    sys0.InitialStates(i).Fixed = false;
end

%% ── Estimate ────────────────────────────────────────────────────────────────
opt = nlgreyestOptions('Display', 'on');
opt.SearchMethod                = 'lm';
opt.SearchOptions.MaxIterations = 300;
opt.SearchOptions.Tolerance     = 0.00065; 

sys_est = nlgreyest(data, sys0, opt);

%% ── Sign check ────────────────────────────────────────────────────────────────
figure;
subplot(2,1,1); plot(t_trimmed, u_trimmed); ylabel('u'); grid on;
subplot(2,1,2); plot(t_trimmed, th1_rad(i_start:i_end)); ylabel('\theta_1 [rad]'); grid on;
xlabel('Time [s]');

%% ── Results ─────────────────────────────────────────────────────────────────
fprintf('\n--- Estimated (free) parameters ---\n');
for i = 1:numel(param_names)
    if ~sys_est.Parameters(i).Fixed
        fprintf('  %-5s = %.6f\n', param_names{i}, sys_est.Parameters(i).Value);
    end
end
fprintf('\n--- Fixed parameters (from pendulum_params) ---\n');
for i = 1:numel(param_names)
    if sys_est.Parameters(i).Fixed
        fprintf('  %-5s = %.6f\n', param_names{i}, sys_est.Parameters(i).Value);
    end
end

figure('Name', 'Compare: measured vs model');
compare(data, sys_est);

figure('Name', 'Residual analysis');
resid(data, sys_est);
