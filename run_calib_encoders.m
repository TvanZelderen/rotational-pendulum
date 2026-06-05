% Encoder calibration — offset and gain for theta1 and theta2.
%
% BEFORE RUNNING:
%   1. In rotpentemplate.slx, temporarily set:
%        theta1 offset block = 0,  theta1 gain block = 1
%        theta2 offset block = 0,  theta2 gain block = 1
%      (so simout reports raw encoder units, not calibrated degrees)
%   2. Run calib.m  — opens fugiboard, resets encoder
%   3. Run hwinit.m — sets interface gains (DAC/ADC, not encoder scale)
%
% Method: two-point calibration per encoder.
%   offset_i = raw_down_i
%   gain_i   = 180 / (raw_up_i - raw_down_i)   [deg per raw-unit]
%
% Cell structure (run cells in order, reposition hardware between cells):
%   Cell A  — both arms hanging down      -> th1down, th2down
%   Cell B1 — arm 1 held inverted (~180°) -> th1up
%   Cell B2 — arm 2 held inverted (~180°) -> th2up   (arm 1 stays down)
%   Cell C  — compute + report
%   Cell D  — optional: slow arm-1 sweep to cross-check gain_th1

%clear; clc;

pendulum_params;
assert(exist('fugihandle', 'var'), ...
    'Run calib.m and hwinit.m before using hardware mode.');

h     = 0.01;   % sample period [s]
trim  = 1.0;    % seconds to cut from each end of capture
controller_sat = 0.5;

%% ── Cell A: both arms hanging down ──────────────────────────────────────────
% Let both arms hang naturally. Do not touch the rig.

run_time = 10;    % [s] — long enough to average out vibration
t = (0 : h : run_time)';
u = zeros(size(t));
simin = [t, u];

sim rotpentemplate;

t_out = simout.Time;
y_raw = simout.Data;

mask = t_out > trim & t_out < run_time - trim;
th1_A = y_raw(mask, 1);   % raw encoder units
th2_A = y_raw(mask, 3);

th1down = mean(th1_A);
th2down = mean(th2_A);

fprintf('\n── Cell A: down-down ──────────────────────────\n');
fprintf('  th1down = %8.4f   (std = %.4f)\n', th1down, std(th1_A));
fprintf('  th2down = %8.4f   (std = %.4f)\n', th2down, std(th2_A));
fprintf('  (std should be near zero — if not, arm is moving)\n');

save_run(simin, simout, 'calib_downdown');

figure(1); clf;
subplot(2,1,1); plot(t_out, y_raw(:,1)); hold on;
    yline(th1down, 'r--', sprintf('mean=%.3f', th1down));
    ylabel('\theta_1 raw'); grid on;
subplot(2,1,2); plot(t_out, y_raw(:,3)); hold on;
    yline(th2down, 'r--', sprintf('mean=%.3f', th2down));
    ylabel('\theta_2 raw'); xlabel('Time [s]'); grid on;
sgtitle('Cell A — down-down (raw encoder)');

%% ── Cell B1: arm 1 held inverted (~180°) ─────────────────────────────────
% Hold arm 1 upright (pointing up, ~180° from down).
% Arm 2 can hang freely — only th1 is read here.

run_time = 5;     % [s] — just need a steady capture
t = (0 : h : run_time)';
u = zeros(size(t));
simin = [t, u];

sim rotpentemplate;

t_out = simout.Time;
y_raw = simout.Data;

mask = t_out > trim & t_out < run_time - trim;
th1_B1 = y_raw(mask, 1);

th1up = mean(th1_B1);

fprintf('\n── Cell B1: arm 1 up ──────────────────────────\n');
fprintf('  th1up   = %8.4f   (std = %.4f)\n', th1up, std(th1_B1));

save_run(simin, simout, 'calib_arm1up');

figure(2); clf;
plot(t_out, y_raw(:,1)); hold on;
yline(th1up, 'r--', sprintf('mean=%.3f', th1up));
ylabel('\theta_1 raw'); xlabel('Time [s]'); grid on;
title('Cell B1 — arm 1 up (raw encoder)');

%% ── Cell B2: arm 2 held inverted (~180°) ─────────────────────────────────
% Arm 1 hangs down. Hold arm 2 upright relative to arm 1 (pendulum inverted).
% Only th2 is read here.

run_time = 5;
t = (0 : h : run_time)';
u = zeros(size(t));
simin = [t, u];

sim rotpentemplate;

t_out = simout.Time;
y_raw = simout.Data;

mask = t_out > trim & t_out < run_time - trim;
th2_B2 = y_raw(mask, 3);

th2up = mean(th2_B2);

fprintf('\n── Cell B2: arm 2 up ──────────────────────────\n');
fprintf('  th2up   = %8.4f   (std = %.4f)\n', th2up, std(th2_B2));

save_run(simin, simout, 'calib_arm2up');

figure(3); clf;
plot(t_out, y_raw(:,3)); hold on;
yline(th2up, 'r--', sprintf('mean=%.3f', th2up));
ylabel('\theta_2 raw'); xlabel('Time [s]'); grid on;
title('Cell B2 — arm 2 up (raw encoder)');

%% ── Cell C: compute calibration + comparison table ──────────────────────
tm_th1down = 2.37;
tm_th1up   = 1.046;
tm_th2down = 1.195;
tm_th2up   = 3.692;

offset_th1 = th1down;
offset_th2 = th2down;
gain_th1   = 180 / (th1up - th1down);    % [deg per raw-unit]
gain_th2   = 180 / (th2up - th2down);

tm_offset_th1 = tm_th1down;
tm_offset_th2 = tm_th2down;
tm_gain_th1   = 180 / (tm_th1up - tm_th1down);
tm_gain_th2   = 180 / (tm_th2up - tm_th2down);

fprintf('\n══════════════════════════════════════════════════════════\n');
fprintf('  Encoder calibration result\n');
fprintf('══════════════════════════════════════════════════════════\n');
fprintf('               ours     other group    diff\n');
fprintf('  th1down  %9.4f    %9.4f    %+.4f\n', th1down,   tm_th1down, th1down   - tm_th1down);
fprintf('  th1up    %9.4f    %9.4f    %+.4f\n', th1up,     tm_th1up,   th1up     - tm_th1up);
fprintf('  th2down  %9.4f    %9.4f    %+.4f\n', th2down,   tm_th2down, th2down   - tm_th2down);
fprintf('  th2up    %9.4f    %9.4f    %+.4f\n', th2up,     tm_th2up,   th2up     - tm_th2up);
fprintf('──────────────────────────────────────────────────────────\n');
fprintf('  offset1  %9.4f    %9.4f    %+.4f\n', offset_th1, tm_offset_th1, offset_th1 - tm_offset_th1);
fprintf('  gain1    %9.4f    %9.4f    %+.4f\n', gain_th1,   tm_gain_th1,   gain_th1   - tm_gain_th1);
fprintf('  offset2  %9.4f    %9.4f    %+.4f\n', offset_th2, tm_offset_th2, offset_th2 - tm_offset_th2);
fprintf('  gain2    %9.4f    %9.4f    %+.4f\n', gain_th2,   tm_gain_th2,   gain_th2   - tm_gain_th2);
fprintf('══════════════════════════════════════════════════════════\n');
fprintf('\nPaste into rotpentemplate.slx:\n');
fprintf('  theta1 offset block: %.4f\n', offset_th1);
fprintf('  theta1 gain block:   %.4f\n', gain_th1);
fprintf('  theta2 offset block: %.4f\n', offset_th2);
fprintf('  theta2 gain block:   %.4f\n', gain_th2);
fprintf('\nExpected ~73 deg/raw-unit per encoder (old: th1=%.1f, th2=%.1f)\n', ...
    360/4.9036, 360/4.9390);

%% ── Cell D (optional): slow arm-1 sweep — cross-check gain_th1 ───────────
% Drive arm 1 slowly through several full revolutions.
% gain_th1_sweep = 360 / (max(th1_raw) - min(th1_raw))  per revolution.
% Dead-zone noise limits precision — treat as sanity check only.
%
% NOTE: skip if arm-1 collisions or wiring risk; Cell C is sufficient.

run_time   = 30;   % [s]
u_amp    = 0.15; % [normalised] — slow enough to stay near terminal velocity

t = (0 : h : run_time)';
u = u_amp * ones(size(t));
simin = [t, u];

sim rotpentemplate;

t_out = simout.Time;
y_raw = simout.Data;

th1_D = y_raw(:, 1);   % raw, un-calibrated

% Rough gain: count zero-crossings to find revolution boundaries, or just use
% total-range / estimated revolutions based on total time and known terminal vel.
raw_range = max(th1_D) - min(th1_D);
% If the arm completed exactly N full turns, gain = N*360 / raw_range.
% Print the raw range; user estimates N from the plot.
fprintf('\n── Cell D: sweep cross-check ──────────────────\n');
fprintf('  raw range of th1: %.4f\n', raw_range);
fprintf('  If arm completed N full turns: gain_th1 = N*360/%.4f\n', raw_range);

figure(4); clf;
plot(t_out, th1_D);
ylabel('\theta_1 raw'); xlabel('Time [s]'); grid on;
title('Cell D — arm 1 slow sweep (count turns visually)');

save_run(simin, simout, 'calib_arm1sweep');
