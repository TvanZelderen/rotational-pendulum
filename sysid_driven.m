clear; clc;
pendulum_params;   % populates struct p — source of truth for guesses + fixed values

%% ── Configuration ──────────────────────────────────────────────────────────
% FIT mode    : set file_name to amp400 run; keep kbc1_fit line below commented.
% VALIDATE mode: set file_name to amp300 run; uncomment kbc1_fit lock line below.
file_name   = '20260605_132001_multisine_amp300.mat';
data_folder = 'data';

trim_start = 1;     % [s] skip initial transient (search starts here)
win_len    = 20;    % [s] sliding window length

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

%% ── Trim: sliding window — find calmest win_len-second stretch ──────────────
win_samples = round(win_len / h_out);
i_lo        = round(trim_start / h_out) + 1;
i_hi        = length(t_sim) - win_samples;

v_combined  = abs(dth1_rad) + abs(dth2_rad);
n_search    = i_hi - i_lo + 1;
win_cost    = zeros(n_search, 1);
for k = 1:n_search
    win_cost(k) = mean(v_combined(i_lo+k-1 : i_lo+k-1+win_samples-1));
end
[~, i_best] = min(win_cost);
i_start     = i_lo + i_best - 1;
i_end       = i_start + win_samples - 1;

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

%% ── Custom 1-D kbc1 fit: residual high-pass cost ────────────────────────────
% Strategy: simulate the ODE with true absolute angles (gravity stays intact),
% high-pass the prediction error at hp_fc to reject sub-0.1 Hz drift, then
% minimise the filtered SSE over kbc1 with fminbnd (only 1 free parameter).
%
% Prefiltering the *data* angles is invalid for this model because θ₁ enters
% the gravity potential (sin θ₁ in the EOM) — removing the DC mean corrupts
% the restoring torque.  Filtering the residual avoids this entirely.

th1_meas = th1_rad(i_start:i_end);
th2_meas = th2_rad(i_start:i_end);
x0_fit   = y_trimmed(1, :)';       % [th1; dth1; th2; dth2] at trim start

hp_fc      = 0.1;                                    % [Hz] residual high-pass corner
[bhp, ahp] = butter(2, hp_fc/(fs_out/2), 'high');
ode_opts   = odeset('RelTol',1e-6,'AbsTol',1e-8);

costfun = @(k) resid_sse(k, p, t_trimmed, u_trimmed, x0_fit, ...
                         th1_meas, th2_meas, bhp, ahp, ode_opts);

% VALIDATE mode: uncomment the next line and set value from a prior fit.
kbc1_fit = 2.9538;   % <-- lock value here; comment out to re-run fminbnd (FIT mode)

if ~exist('kbc1_fit', 'var')
    fprintf('\nFitting kbc1 via fminbnd (residual high-passed at %.2f Hz)...\n', hp_fc);
    opt_fb = optimset('Display','iter','TolX',1e-4);
    [kbc1_fit, J_fit] = fminbnd(costfun, 0.5, 5.0, opt_fb);
    fprintf('\nFitted  kbc1 = %.4f N·m·s/rad  (cost J = %.4g)\n', kbc1_fit, J_fit);
else
    fprintf('\nValidate mode — kbc1 locked to %.4f N·m·s/rad\n', kbc1_fit);
    J_fit = costfun(kbc1_fit);
    fprintf('Cost at locked value: J = %.4g\n', J_fit);
end
fprintf('Reference values:  step-ID ~ 2.44,  terminal-vel ~ 2.37\n');

%% ── Re-simulate at fitted kbc1 ──────────────────────────────────────────────
odef_fit = @(t_ode, x_ode) rotpen_ode_idnlgrey(t_ode, x_ode, ...
    interp1(t_trimmed, u_trimmed, t_ode, 'linear', 'extrap'), ...
    p.km, kbc1_fit, p.c2, p.J1, p.J2, p.l1, p.l2, p.lc1, p.m1, p.m2, p.g);
[~, xs_fit] = ode45(odef_fit, t_trimmed, x0_fit, ode_opts);

th1_model = xs_fit(:,1);
th2_model = xs_fit(:,3);

e1_raw = th1_model - th1_meas;
e2_raw = th2_model - th2_meas;
e1_hp  = filtfilt(bhp, ahp, e1_raw);
e2_hp  = filtfilt(bhp, ahp, e2_raw);

%% ── Results: time overlay ───────────────────────────────────────────────────
figure('Name', 'Fit result — model vs measured');
subplot(3,1,1);
    plot(t_trimmed, rad2deg(th1_meas),  'Color',[0.7 0.7 0.7]); hold on;
    plot(t_trimmed, rad2deg(th1_model), 'b', 'LineWidth', 1.2);
    ylabel('\theta_1 [deg]'); legend('Measured','Model'); grid on;
    title(sprintf('kbc1 = %.4f N·m·s/rad  (hp\\_fc = %.2f Hz)', kbc1_fit, hp_fc));
subplot(3,1,2);
    plot(t_trimmed, rad2deg(th2_meas),  'Color',[0.7 0.7 0.7]); hold on;
    plot(t_trimmed, rad2deg(th2_model), 'r', 'LineWidth', 1.2);
    ylabel('\theta_2 [deg]'); legend('Measured','Model'); grid on;
subplot(3,1,3);
    plot(t_trimmed, u_trimmed, 'k');
    ylabel('u [-]'); xlabel('Time [s]'); grid on;

%% ── Results: residual — raw vs high-passed ───────────────────────────────────
figure('Name', 'Residuals — raw vs high-passed (drift visible)');
subplot(2,1,1);
    plot(t_trimmed, rad2deg(e1_raw), 'Color',[0.7 0.7 0.7]); hold on;
    plot(t_trimmed, rad2deg(e1_hp),  'b', 'LineWidth', 1.2);
    yline(0, 'k:');
    ylabel('e(\theta_1) [deg]'); legend('Raw residual','High-passed (cost)'); grid on;
    title(sprintf('Residuals — drift below %.2f Hz removed from cost', hp_fc));
subplot(2,1,2);
    plot(t_trimmed, rad2deg(e2_raw), 'Color',[0.7 0.7 0.7]); hold on;
    plot(t_trimmed, rad2deg(e2_hp),  'r', 'LineWidth', 1.2);
    yline(0, 'k:');
    ylabel('e(\theta_2) [deg]'); legend('Raw residual','High-passed (cost)'); grid on;
    xlabel('Time [s]');

%% ════════════════════════════════════════════════════════════════════════════
%  Local functions  (must appear after all script code)
% ════════════════════════════════════════════════════════════════════════════

function J = resid_sse(kbc1_try, p, tt, ut, x0, y1, y2, bhp, ahp, oo)
% RESID_SSE  Cost for fminbnd: simulate ODE, high-pass residual, sum squares.
%
%   kbc1_try — trial damping value [N·m·s/rad]
%   p        — parameter struct (all other params fixed)
%   tt       — time vector matching y1, y2 [s]
%   ut       — input vector at tt [-]
%   x0       — initial state [th1; dth1; th2; dth2]
%   y1, y2   — measured th1, th2 [rad]
%   bhp,ahp  — high-pass filter coefficients (Butterworth)
%   oo       — odeset options

    odef = @(tq, xq) rotpen_ode_idnlgrey(tq, xq, ...
        interp1(tt, ut, tq, 'linear', 'extrap'), ...
        p.km, kbc1_try, p.c2, p.J1, p.J2, p.l1, p.l2, p.lc1, p.m1, p.m2, p.g);
    [~, xs] = ode45(odef, tt, x0, oo);

    e1 = filtfilt(bhp, ahp, xs(:,1) - y1);   % th1 residual, drift removed
    e2 = filtfilt(bhp, ahp, xs(:,3) - y2);   % th2 residual, drift removed
    J  = sum(e1.^2 + e2.^2);
end
