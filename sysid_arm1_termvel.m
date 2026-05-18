% Offline arm-1 terminal-velocity sweep analysis.
% Collect data first with run_arm1_terminal_vel.m.
% Identifies: kbc1 (composite back-EMF + joint damping) and tauc_kinetic
% (Coulomb kinetic friction torque).
%
% Requires km to already be set in pendulum_params.m (from sysid_arm1_ramp.m).

clear; clc;
pendulum_params;

%% ── Configuration ───────────────────────────────────────────────────────────
file_name    = '20260518_115759_arm1_termvel_10steps.mat';         % e.g. '20260518_100000_arm1_termvel_8steps.mat'
data_folder  = 'data';

trim_start   = 1;          % [s] skip any startup transient at the very beginning
trim_end     = Inf;        % [s] Inf = use full run

% What fraction of each constant-u segment counts as "steady state"?
% Terminal velocity is reached after ~3 time constants; the last 40% of a
% t_step=8s window is safely past that for most expected kbc1 values.
settle_frac  = 0.4;

% Minimum segment length to accept (shorter segments may not have settled)
min_seg_s    = 3;          % [s]

% Threshold on |du| to detect step transitions (tune if edges are missed)
edge_thresh  = 0.05;       % [normalised u]

% %% Intermission
% 
% run_data = load('data/20260518_115759_arm1_termvel_10steps.mat');
% y = run_data.simout.Data;
% th1_raw = y(:,1);
% % Where exactly is the flat section in deg?
% figure; plot(mod(th1_raw, 360)); ylabel('\theta_1 mod 360 [deg]'); grid on;
% % The flat section should cluster around THETA_DEAD

%% ── Load data ────────────────────────────────────────────────────────────────
run_data = load(fullfile(data_folder, file_name));
t_raw  = run_data.simin(:, 1);
u_raw  = run_data.simin(:, 2);
y_raw  = run_data.simout.Data;   % [N×5]: [th1_deg, dth1_dps, th2_deg, dth2_dps, psi_deg]
h      = mean(diff(t_raw));

[th1_deg, dth1_dps] = unwrap_simout(run_data.simout, 0, h);
th1_raw  = deg2rad(th1_deg);
dth1_raw = deg2rad(dth1_dps);

%% ── Unwrap diagnostics ───────────────────────────────────────────────────────
% Four-panel figure: raw vs patched th1, bad mask, and derivative comparison.
% Zoom in on the dead-zone crossing to see what's happening sample by sample.
THETA_DEAD_DBG = 82.1813;
TOL_DBG        = 8;

t_so           = run_data.simout.Time;                    % simout time vector
th1_orig_deg   = y_raw(:,1);                              % raw encoder [deg]
th1_orig_rad   = deg2rad(th1_orig_deg);
dth1_orig      = [0; diff(th1_orig_rad)] / h;             % raw diff [rad/s]
dth1_col2      = deg2rad(y_raw(:,2));                     % Simulink derivative [rad/s]

th1_mod_raw    = mod(th1_orig_deg - THETA_DEAD_DBG, 360);
near_dead_dbg  = th1_mod_raw < TOL_DBG | th1_mod_raw > 360 - TOL_DBG;
bad_dbg        = near_dead_dbg | [false; diff(near_dead_dbg(:)) < 0];

% col2 from Simulink has HW derivative overflow spikes (~1e10); clip for display only
PLOT_CLIP      = 80;    % [rad/s]
dth1_col2_c    = max(min(dth1_col2, PLOT_CLIP), -PLOT_CLIP);

figure('Name', 'Unwrap diagnostics — full run');
subplot(4,1,1);
  plot(t_so, th1_orig_deg, 'Color',[0.7 0.7 0.7]); hold on;
  plot(t_so, th1_deg, 'b');
  ylabel('\theta_1 [deg]'); legend('Raw','Patched'); grid on;
  title('th1: raw (grey) vs unwrap\_simout output (blue)');
subplot(4,1,2);
  plot(t_so, bad_dbg, 'k'); ylabel('bad mask'); ylim([-0.1 1.1]); grid on;
  title(sprintf('Bad mask — %d samples patched (%d windows)', ...
        sum(bad_dbg), sum(diff([0; bad_dbg(:)]) > 0)));
subplot(4,1,3);
  plot(t_so, dth1_orig,    'Color',[0.8 0.8 0.8]); hold on;
  plot(t_so, dth1_col2_c, 'g');
  plot(t_so, dth1_raw,    'b');
  ylabel('d\theta_1 [rad/s]');
  legend('diff(raw th1)/h',sprintf('col2 (clipped ±%d)',PLOT_CLIP),'unwrap\_simout'); grid on;
  title('Derivative: three sources');
subplot(4,1,4);
  plot(t_so, dth1_col2_c - dth1_raw, 'r');
  ylabel('\Delta d\theta_1 [rad/s]'); xlabel('Time [s]'); grid on;
  title('col2 − unwrap\_simout derivative (should be ~0 except at crossings)');

% Zoom in on first dead-zone crossing
first_bad = find(bad_dbg, 1);
if ~isempty(first_bad)
    zoom_win = max(1, first_bad-50) : min(numel(t_so), first_bad+150);
    figure('Name', 'Unwrap diagnostics — crossing zoom');
    subplot(3,1,1);
      plot(t_so(zoom_win), th1_orig_deg(zoom_win), 'ko-', 'MarkerSize', 4); hold on;
      plot(t_so(zoom_win), th1_deg(zoom_win),      'bs-', 'MarkerSize', 4);
      ylabel('\theta_1 [deg]'); legend('Raw','Patched'); grid on;
      title('Zoom: first crossing (sample by sample)');
    subplot(3,1,2);
      plot(t_so(zoom_win), bad_dbg(zoom_win), 'k'); ylim([-0.1 1.1]); grid on;
      ylabel('bad'); title('Bad mask in window');
    subplot(3,1,3);
      plot(t_so(zoom_win), dth1_orig(zoom_win),    'ko-', 'MarkerSize',4); hold on;
      plot(t_so(zoom_win), dth1_col2_c(zoom_win), 'g.-', 'MarkerSize',8);
      plot(t_so(zoom_win), dth1_raw(zoom_win),    'bs-', 'MarkerSize',4);
      ylabel('d\theta_1 [rad/s]'); legend('diff(raw)/h','col2','patched'); grid on;
      title('Derivative comparison in window');
end

%% ── Trim ─────────────────────────────────────────────────────────────────────
i_start = round(trim_start / h) + 1;
i_end   = min(round(trim_end / h) + 1, length(t_raw));

t        = t_raw(i_start:i_end);
u        = u_raw(i_start:i_end);
th1_raw  = th1_raw(i_start:i_end);
dth1_raw = dth1_raw(i_start:i_end);

fprintf('Loaded: %.1f s of data at h = %.3f s\n', t(end)-t(1), h);

%% ── Filter ───────────────────────────────────────────────────────────────────
fs = 1/h;
fc = 5;                    % cutoff [Hz] — smooth velocity without lagging the steps
[b_f, a_f] = butter(2, fc/(fs/2));
dth1_f = filtfilt(b_f, a_f, dth1_raw);

figure('Name', 'Filter check — dth1');
plot(t, dth1_raw, 'Color', [0.8 0.8 0.8]); hold on;
plot(t, dth1_f, 'r', 'LineWidth', 1.5);
ylabel('d\theta_1 [rad/s]'); xlabel('Time [s]');
legend('Raw', 'Filtered'); title('Filter verification'); grid on;

%% ── Overview plot ────────────────────────────────────────────────────────────
figure('Name', 'Terminal-velocity overview');
subplot(3,1,1); plot(t, u);      ylabel('u [-]');           grid on; title('Input staircase');
subplot(3,1,2); plot(t, dth1_f); ylabel('d\theta_1 [rad/s]'); xlabel('Time [s]'); grid on;
subplot(3,1,3); plot(t, th1_raw); ylabel('theta_1 [rad]'); xlabel('Time [s]'); grid on;

%% ── Step segmentation ────────────────────────────────────────────────────────
% Detect step edges from the raw (unfiltered) input — transitions are sharp
% between samples, so diff(u) spikes exactly at the edge.
u_jumps = [0; abs(diff(u))];          % prepend 0 so length matches
edges   = find(u_jumps > edge_thresh); % indices where a new step begins
edges   = [1; edges; length(t)+1];    % bookend with start and one-past-end

u_levels = [];
omega_ss  = [];

for k = 1:length(edges)-1
    i0      = edges(k);
    i1      = edges(k+1) - 1;
    seg_len = i1 - i0 + 1;

    % Reject segments too short to have settled
    if seg_len < round(min_seg_s / h)
        continue;
    end

    % Steady-state window: last settle_frac of the segment
    i_ss   = round(i0 + (1 - settle_frac)*seg_len) : i1;
    u_mean = mean(u(i_ss));
    w_mean = mean(dth1_f(i_ss));

    % Skip segments where the arm is barely moving (near u=0 or stiction held it)
    if abs(u_mean) < edge_thresh
        continue;
    end

    u_levels(end+1, 1) = u_mean;   %#ok<AGROW>
    omega_ss(end+1, 1) = w_mean;   %#ok<AGROW>
end

fprintf('Extracted %d (u, omega_ss) pairs.\n', length(u_levels));
if length(u_levels) < 2
    error('Too few pairs — check edge_thresh or min_seg_s, or check that the run has multiple u levels.');
end

%% ── Fit: kbc1 and tauc_kinetic ───────────────────────────────────────────────
% At terminal velocity, ddth1 = 0 and gravity averages out over full rotations.
% The averaged arm-1 EOM reduces to:
%
%   km * u  =  kbc1 * omega_ss  +  tauc_kinetic * sign(omega_ss)
%
% This is linear in the two unknowns [kbc1, tauc_kinetic]:
%
%   A * [kbc1; tauc_kinetic]  =  b
%
% where:
%   A = [omega_ss,  sign(omega_ss)]   (N×2 regressor matrix)
%   b = -p.km * u_levels              (N×1 right-hand side; minus because rotpen_ode uses tau = -km·u)

A = [omega_ss, sign(omega_ss)];
b = -p.km * u_levels;

% Least-squares solve: overdetermined for N>2, exact for N=2
theta = A \ b;

kbc1         = theta(1);   % [N·m·s/rad]
tauc_kinetic = theta(2);   % [N·m]

% Goodness of fit — R² on the torque residuals
b_hat  = A * theta;
SS_res = sum((b - b_hat).^2);
SS_tot = sum((b - mean(b)).^2);
R2     = 1 - SS_res / SS_tot;

fprintf('R² = %.4f  (>0.95 expected for clean data)\n', R2);
if R2 < 0.9
    warning('Low R² — check for asymmetric friction or non-viscous effects.');
end

%% ── Plots ────────────────────────────────────────────────────────────────────
% Fitted line evaluated over a dense omega grid (handles the sign discontinuity)
tau_meas = -p.km * u_levels;   % motor torque consistent with rotpen_ode: tau = -km*u
if any(omega_ss > 0)
    omega_pos = linspace(0, max(omega_ss(omega_ss>0))+0.1, 100)';
    b_pos = kbc1*omega_pos + tauc_kinetic;
else
    omega_pos = []; b_pos = [];
end
if any(omega_ss < 0)
    omega_neg = linspace(min(omega_ss(omega_ss<0))-0.1, 0, 100)';
    b_neg = kbc1*omega_neg - tauc_kinetic;
else
    omega_neg = []; b_neg = [];
end

figure('Name', 'Terminal-velocity regression');
subplot(2,1,1); hold on;
  % Measured (tau, omega_ss) pairs plotted as torque vs velocity
  plot(omega_ss(omega_ss>0), tau_meas(omega_ss>0), 'bo', 'MarkerFaceColor','b');
  plot(omega_ss(omega_ss<0), tau_meas(omega_ss<0), 'rs', 'MarkerFaceColor','r');
  if ~isempty(omega_pos); plot(omega_pos, b_pos, 'b-', 'LineWidth', 1.5); end
  if ~isempty(omega_neg); plot(omega_neg, b_neg, 'r-', 'LineWidth', 1.5); end
  xlabel('\omega_{ss} [rad/s]'); ylabel('\tau_{motor} = -k_m u  [N\cdotm]');
  legend('Data (+\omega)', 'Data (-\omega)', 'Fit (+\omega)', 'Fit (-\omega)');
  title(sprintf('kbc1 = %.4f  N·m·s/rad,   tauc\\_kinetic = %.4f  N·m,   R² = %.3f', ...
      kbc1, tauc_kinetic, R2));
  grid on;
subplot(2,1,2);
  plot(omega_ss, b - b_hat, 'ko', 'MarkerFaceColor','k');
  yline(0, '--k'); yline(2*std(b-b_hat), '--', '+2\sigma'); yline(-2*std(b-b_hat), '--', '-2\sigma');
  xlabel('\omega_{ss} [rad/s]'); ylabel('Residual [N\cdotm]');
  title('Residuals — should be small and unsystematic'); grid on;

%% ── Results ─────────────────────────────────────────────────────────────────
fprintf('\n--- Arm-1 terminal-velocity identification ---\n');
fprintf('  kbc1         = %.6f  N·m·s/rad\n', kbc1);
fprintf('  tauc_kinetic = %.6f  N·m\n', tauc_kinetic);
fprintf('\nUpdate pendulum_params.m, then validate with sysid_arm1_driven.m.\n');
