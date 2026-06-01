% Arm-1 step-response identification — J1 and kbc1 cross-check.
% Loads output from run_step_id.m (5-level staircase; each level rises from rest).
%
% Method: for each step segment fit a first-order exponential to the dth1 rise.
%   dth1(t) = w_ss * (1 - exp(-(t-t0)/tau))
% From the time constant tau and the known kbc1:
%   J1 = tau * kbc1          <-- TODO: derive this from the arm-1 EOM at rest
% Cross-check: at steady state ddth1=0, so:
%   kbc1_check = -km * u / w_ss  <-- TODO: derive from EOM at terminal velocity
%
% Run sysid_termvel.m first to confirm kbc1 (used as input here).
%
% Prerequisites: pendulum_params.m populated with km and kbc1.

clear; clc;
pendulum_params;

%% ── Configuration ────────────────────────────────────────────────────────────
file_name   = '20260527_130529_arm1_stepid_5levels.mat';
data_folder = 'data';

trim_start  = 0.5;       % [s] skip any startup glitch

edge_thresh = 0.05;      % |du| threshold to detect step transitions [normalised u]
min_seg_s   = 2;         % [s] minimum segment length to consider
settle_frac = 0.4;       % fraction of segment treated as steady state (same as termvel)

%% ── Load + clean signal ──────────────────────────────────────────────────────
% Use simout.Time as master — simin and simout can differ in length.
% Interpolate u from simin onto simout grid (same pattern as sysid_driven.m:27-31).
run_data = load(fullfile(data_folder, file_name));
t_sim = run_data.simout.Time;
h     = mean(diff(t_sim));
u_sim = interp1(run_data.simin(:,1), run_data.simin(:,2), t_sim, 'linear', 'extrap');

[~, dth1_dps] = unwrap_simout(run_data.simout, 0, h);
dth1_raw = deg2rad(dth1_dps);   % [rad/s]

% Apply filter (same chain as sysid_termvel — medfilt + Butterworth)
fs = 1/h;
fc = 5;
[b_f, a_f] = butter(2, fc/(fs/2));
dth1_f = filtfilt(b_f, a_f, medfilt1(dth1_raw, 7));

% Trim — all arrays share t_sim as index base
i_start = round(trim_start / h) + 1;
t       = t_sim(i_start:end);
u       = u_sim(i_start:end);
dth1_f  = dth1_f(i_start:end);

fprintf('Loaded: %.1f s  |  h = %.3f s\n', t(end)-t(1), h);

%% ── Overview ─────────────────────────────────────────────────────────────────
figure('Name', 'Step-ID overview');
subplot(2,1,1); stairs(t, u);     ylabel('u [-]'); grid on; title('Input');
subplot(2,1,2); plot(t, dth1_f);  ylabel('d\theta_1 [rad/s]'); xlabel('Time [s]'); grid on;

%% ── Segment by input edges ───────────────────────────────────────────────────
% Reuse the same edge-detection pattern as sysid_termvel.
u_jumps = [0; abs(diff(u))];
edges   = find(u_jumps > edge_thresh);
edges   = [1; edges; length(t)+1];   % bookend

% Collect non-zero step segments (skip zero-dwell and too-short segments)
step_segs = [];   % each row: [i0, i1, u_level]

for k = 1:length(edges)-1
    i0      = edges(k);
    i1      = edges(k+1) - 1;
    seg_len = i1 - i0 + 1;

    if seg_len < round(min_seg_s / h); continue; end

    i_ss   = round(i0 + (1-settle_frac)*seg_len) : i1;
    u_mean = mean(u(i_ss));

    if abs(u_mean) < edge_thresh; continue; end   % zero dwell — skip

    step_segs(end+1, :) = [i0, i1, u_mean];  %#ok<AGROW>
end

n_steps = size(step_segs, 1);
fprintf('Found %d non-zero step segments.\n', n_steps);
if n_steps < length(unique(u_sim(u_sim > edge_thresh))) - 1
    fprintf('  (some levels missing — likely arm not at rest at step start; proceeding with available levels)\n');
end

%% ── Per-step exponential fit ─────────────────────────────────────────────────
J1_est   = NaN(n_steps, 1);
kbc1_est = NaN(n_steps, 1);
tau_est  = NaN(n_steps, 1);
wss_est  = NaN(n_steps, 1);

n_cols = ceil(sqrt(n_steps));
n_rows = ceil(n_steps / n_cols);
figure('Name', 'Per-step exponential fits');

for k = 1:n_steps
    i0    = step_segs(k, 1);
    i1    = step_segs(k, 2);
    u_lev = step_segs(k, 3);

    t_seg = t(i0:i1) - t(i0);   % local time, starts at 0
    w_seg = dth1_f(i0:i1);

    % Steady-state estimate from the last settle_frac of the segment
    i_ss  = round((1-settle_frac)*length(t_seg)) : length(t_seg);
    w_ss0 = mean(w_seg(i_ss));   % initial guess for w_ss

    fun  = @(params, t_in) params(1) * (1 - exp(-t_in / params(2)));
    p0   = [w_ss0, 0.25];      % [w_ss, tau] — tau guess ~0.25s from J1≈0.65/kbc1
    lb   = [-Inf,  0.005];     % tau lower bound: 2× sample period
    ub   = [0,     Inf];       % w_ss must be negative (CW rotation)
    opts = optimoptions('lsqcurvefit', 'Display', 'off');
    pfit = lsqcurvefit(fun, p0, t_seg, w_seg, lb, ub, opts);

    tau_est(k) = pfit(2);
    wss_est(k) = pfit(1);

    % From the arm-1 EOM solution: tau = J1/kbc1  →  J1 = tau * kbc1
    % At steady state (w' = 0, gravity neglected): km*u = kbc1*w_ss  →  kbc1 = -km*u/w_ss
    J1_est(k)   = tau_est(k) * p.kbc1;
    kbc1_est(k) = -p.km * u_lev / wss_est(k);

    % Overlay plot (will update once TODOs are filled)
    subplot(n_rows, n_cols, k); hold on;
    plot(t_seg, w_seg, 'b', 'DisplayName', 'Measured');
    if ~isnan(tau_est(k))
        w_fit = wss_est(k) * (1 - exp(-t_seg / tau_est(k)));
        plot(t_seg, w_fit, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Fit');
    end
    title(sprintf('u = %.2f', u_lev));
    ylabel('[rad/s]'); xlabel('t [s]'); grid on; legend;
end
sgtitle('Per-step dth1 with exponential fit  (red = fit, blue = measured)');

%% ── Aggregate ────────────────────────────────────────────────────────────────
fprintf('\n--- Per-level results ---\n');
fprintf('  Level     tau [s]     J1 [kg·m²]   kbc1_check [N·m·s/rad]\n');
for k = 1:n_steps
    fprintf('  u=%.2f    %.4f      %.6f     %.4f\n', ...
        step_segs(k,3), tau_est(k), J1_est(k), kbc1_est(k));
end

J1_mean   = mean(J1_est,   'omitnan');
kbc1_mean = mean(kbc1_est, 'omitnan');
J1_std    = std(J1_est,    0, 'omitnan');

fprintf('\n  J1   mean = %.6f  kg·m²   (std = %.6f)\n', J1_mean, J1_std);
fprintf('  kbc1 cross-check mean = %.4f  N·m·s/rad  (staircase: %.4f)\n', ...
        kbc1_mean, p.kbc1);

if J1_std / J1_mean > 0.1
    warning('J1 spread > 10%% across levels — check for speed-dependent friction or model mismatch.');
end

%% ── Summary figure ───────────────────────────────────────────────────────────
u_levels_plot = step_segs(:, 3);

figure('Name', 'J1 and kbc1 cross-check across u levels');
subplot(2,1,1);
    scatter(u_levels_plot, J1_est, 60, 'ko', 'filled'); hold on;
    yline(J1_mean, 'r--', sprintf('mean = %.4f', J1_mean));
    ylabel('J_1 [kg·m²]'); xlabel('u level [-]');
    title('J_1 estimate per step'); grid on;
subplot(2,1,2);
    scatter(u_levels_plot, kbc1_est, 60, 'bs', 'filled'); hold on;
    yline(kbc1_mean, 'b--', sprintf('cross-check mean = %.4f', kbc1_mean));
    yline(p.kbc1, 'k:', sprintf('staircase = %.4f', p.kbc1));
    ylabel('kbc1 [N·m·s/rad]'); xlabel('u level [-]');
    title('kbc1 cross-check vs staircase'); grid on;

%% ── Next step ────────────────────────────────────────────────────────────────
fprintf('\n--- Next step ---\n');
fprintf('Update pendulum_params.m:\n');
fprintf('  p.J1 = %.6f;  %% kg·m^2 -- step-response ID, %s\n', J1_mean, datestr(now,'yyyy-mm-dd'));
fprintf('If kbc1_check differs from staircase by >10%%, flag for nlgreyest.\n');
