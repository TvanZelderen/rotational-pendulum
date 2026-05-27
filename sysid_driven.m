clear; clc;
pendulum_params;   % populates struct p — source of truth for guesses + fixed values

%% ── Configuration ──────────────────────────────────────────────────────────
file_name   = '20260527_100747_multisine_amp800.mat';
data_folder = 'data';

trim_start = 1;     % [s] skip initial transient
trim_end   = 28;    % [s]

%% ── Load data ───────────────────────────────────────────────────────────────
run_data = load(fullfile(data_folder, file_name));
t_raw  = run_data.simin(:, 1);
u_raw  = run_data.simin(:, 2);
y_raw  = run_data.simout.Data;   % [N×5]: [th1_deg, dth1_dps, th2_deg, dth2_dps, psi_deg]
h      = mean(diff(t_raw));
f_min  = 0.2;   % multisine band [Hz]
f_max  = 5.0;

[th1_deg, dth1_dps, th2_deg, dth2_dps] = unwrap_simout(run_data.simout, 0, h);
th1_rad  = deg2rad(th1_deg);
dth1_rad = deg2rad(dth1_dps);
th2_rad  = deg2rad(th2_deg);
dth2_rad = deg2rad(dth2_dps);

% simout and simin may have different lengths; use simout's time as master
t_sim  = run_data.simout.Time;
h_out  = mean(diff(t_sim));    % simout sample interval — may differ from simin
fs_out = 1/h_out;
u_sim  = interp1(t_raw, u_raw, t_sim, 'linear', 'extrap');

fs = 1/h;

%% ── Trim: find calmest start in window ──────────────────────────────────────
i_start = round(trim_start / h_out) + 1;
i_end   = min(round(trim_end  / h_out) + 1, length(t_sim));

v_combined = abs(dth1_rad(i_start:i_end)) + abs(dth2_rad(i_start:i_end));
[~, i_min] = min(v_combined);
i_start    = i_start + i_min - 1;

y_trimmed = [th1_rad(i_start:i_end),  dth1_rad(i_start:i_end), ...
             th2_rad(i_start:i_end),  dth2_rad(i_start:i_end)];
u_trimmed  = u_sim(i_start:i_end);
t_trimmed  = t_sim(i_start:i_end);

fprintf('Trim window: t = %.2f to %.2f s  (%d samples)\n', ...
        t_trimmed(1), t_trimmed(end), length(t_trimmed));

%% ── Time domain ─────────────────────────────────────────────────────────────
figure('Name', 'Time domain — trimmed window');
subplot(3,1,1);
    plot(t_trimmed, rad2deg(th1_rad(i_start:i_end)), 'b', ...
         t_trimmed, rad2deg(th2_rad(i_start:i_end)), 'r');
    legend('\theta_1', '\theta_2'); ylabel('[deg]'); grid on;
    title('Angles — trimmed window');
subplot(3,1,2);
    plot(t_trimmed, rad2deg(dth1_rad(i_start:i_end)), 'b', ...
         t_trimmed, rad2deg(dth2_rad(i_start:i_end)), 'r');
    legend('d\theta_1', 'd\theta_2'); ylabel('[deg/s]'); grid on;
subplot(3,1,3);
    plot(t_trimmed, u_trimmed, 'k');
    ylabel('u [-]'); xlabel('Time [s]'); grid on;

%% ── Frequency analysis ───────────────────────────────────────────────────────

% u: full simin record — preserves designed tone spacing (df = 1/Tsim)
n_u    = length(u_raw);
f_u    = (0:floor(n_u/2)-1) * (fs/n_u);
amp_u  = 2*abs(fft(detrend(u_raw), n_u)) / n_u;

% outputs: trimmed simout data
n      = length(t_trimmed);
n_half = floor(n/2);
f_out  = (0:n_half-1) * (fs_out/n);

fft_amp = @(x) 2*abs(fft(detrend(x), n)) / n;

amp_th1  = fft_amp(th1_rad(i_start:i_end));
amp_dth1 = fft_amp(dth1_rad(i_start:i_end));
amp_th2  = fft_amp(th2_rad(i_start:i_end));
amp_dth2 = fft_amp(dth2_rad(i_start:i_end));

% designed multisine tone frequencies (from input design: every other bin)
Tsim_in = t_raw(end);
df_in   = 1 / Tsim_in;
f_tones = (round(f_min/df_in) : 2 : round(f_max/df_in)) * df_in;

figure('Name', 'FFT — full band');
subplot(5,1,1); stem(f_u,   amp_u(1:floor(n_u/2)),    'filled', 'MarkerSize', 2); ylabel('u');    grid on; title('FFT (full band, detrended)');
subplot(5,1,2); stem(f_out, amp_th1(1:n_half),  'filled', 'MarkerSize', 2); ylabel('th1');  grid on;
subplot(5,1,3); stem(f_out, amp_dth1(1:n_half), 'filled', 'MarkerSize', 2); ylabel('dth1'); grid on;
subplot(5,1,4); stem(f_out, amp_th2(1:n_half),  'filled', 'MarkerSize', 2); ylabel('th2');  grid on;
subplot(5,1,5); stem(f_out, amp_dth2(1:n_half), 'filled', 'MarkerSize', 2); ylabel('dth2'); xlabel('Frequency [Hz]'); grid on;

figure('Name', 'FFT — multisine band zoom');
subplot(3,1,1);
    stem(f_u, amp_u(1:floor(n_u/2)), 'filled', 'MarkerSize', 2); ylabel('u [-]'); grid on;
    title(sprintf('FFT zoomed %.1f–%.1f Hz', f_min, f_max)); xlim([f_min f_max]);
subplot(3,1,2);
    stem(f_out, amp_th1(1:n_half), 'filled', 'MarkerSize', 2); ylabel('th1 [rad]'); grid on;
    xline(f_tones, 'r:', 'Alpha', 0.5); xlim([f_min f_max]);
subplot(3,1,3);
    stem(f_out, amp_th2(1:n_half), 'filled', 'MarkerSize', 2); ylabel('th2 [rad]'); grid on;
    xline(f_tones, 'r:', 'Alpha', 0.5); xlim([f_min f_max]);
    xlabel('Frequency [Hz]');

% plot(t_trimmed, y_trimmed(:,1))

% figure; plot(t_trimmed, u_trimmed/max(abs(u_trimmed)), 'b', t_trimmed, th1_rad(i_start:i_end)/max(abs(th1_rad(i_start:i_end))), 'r');
% legend('u (normalised)', 'th1 (normalised)'); grid on;
% title('Sign check — do they go the same direction?');

%% 1. Design the Filter
fc = 5;                     % Cutoff frequency (Hz)
[b, a] = butter(2, fc/(fs/2)); % 2nd order Butterworth

%% 2. Apply Zero-Phase Filtering
% We use 'filtfilt' instead of 'filter' to ensure there is 0ms time delay.
th1_f  = filtfilt(b, a, th1_rad);
dth1_f = filtfilt(b, a, dth1_rad);
th2_f  = filtfilt(b, a, th2_rad);
dth2_f = filtfilt(b, a, dth2_rad);

%% 3. Diagnostic Check
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

%% ── Build idnlgrey model — validation only (all params fixed) ────────────────
% Parameter order: km, kbc1, c2, J1, J2, l1, l2, lc1, m1, m2, g
% (must match rotpen_ode_idnlgrey.m argument list — 11 params)
param_cell = {p.km; p.kbc1; p.c2; p.J1; p.J2; p.l1; p.l2; p.lc1; p.m1; p.m2; p.g};
x0_est     = y_trimmed(1, :)';

sys_val = idnlgrey('rotpen_ode_idnlgrey', [2 1 4], param_cell, x0_est, 0);

param_names = {'km','kbc1','c2','J1','J2','l1','l2','lc1','m1','m2','g'};

for i = 1:numel(param_names)
    sys_val.Parameters(i).Name  = param_names{i};
    sys_val.Parameters(i).Fixed = true;   % validation
end

for i = 1:4
    sys_val.InitialStates(i).Fixed = true;   % fixed to measured x0_est
end

%% ── Build idnlgrey model — fit (J1, kbc1 floating) ────────────────
% Parameter order: km, kbc1, c2, J1, J2, l1, l2, lc1, m1, m2, g
% (must match rotpen_ode_idnlgrey.m argument list — 11 params)
param_cell = {p.km; p.kbc1; p.c2; p.J1; p.J2; p.l1; p.l2; p.lc1; p.m1; p.m2; p.g};
x0_est     = y_trimmed(1, :)';

sys_fit = idnlgrey('rotpen_ode_idnlgrey', [2 1 4], param_cell, x0_est, 0);

param_names = {'km','kbc1','c2','J1','J2','l1','l2','lc1','m1','m2','g'};

for i = 1:numel(param_names)
    sys_fit.Parameters(i).Name  = param_names{i};
    free_params = {'kbc1', 'J1'};
    if ismember(param_names{i}, free_params)
        sys_fit.Parameters(i).Fixed = false;
    else
        sys_fit.Parameters(i).Fixed = true;
    end
end


for i = [1, 3]
    sys_fit.InitialStates(i).Fixed = true;
for i = [2, 4]
    sys_fit.InitialStates(i).Fixed = false;   % floating as dth's cannot be trusted.
end

%% ── Results ─────────────────────────────────────────────────────────────────
fprintf('\n--- Parameters used for validation (all from pendulum_params) ---\n');
for i = 1:numel(param_names)
    fprintf('  %-5s = %.6f\n', param_names{i}, sys_val.Parameters(i).Value);
end

figure('Name', 'Compare: measured vs model');
compare(data, sys_val);

figure('Name', 'Residual analysis');
resid(data, sys_val);
