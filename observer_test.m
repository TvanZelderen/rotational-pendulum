clear; clc;
pendulum_params;   % populates struct p — source of truth for guesses + fixed values

%% ── Configuration ──────────────────────────────────────────────────────────
file_name   = '20260527_123500_arm1_pre_id_f300mHz_a30.mat';
data_folder = 'data';

trim_start = 1;     % [s] skip initial transient
trim_end   = 28;    % [s]

%% ── Load data ───────────────────────────────────────────────────────────────
run_data = load(fullfile(data_folder, file_name));
t_raw  = run_data.simin(:, 1);
u_raw  = run_data.simin(:, 2);
y_raw  = run_data.simout.Data;   % [N×6]: [th1_deg, dth1_dps, th2_deg, dth2_dps, psi_deg, x_hat]
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

%% Extract observer states from simout
% Assuming simout column order: [th1, dth1, th2, dth2, phi, xhat1, xhat2, xhat3, xhat4]
% Adjust indices based on what the column check prints above

xhat_th1  = deg2rad(y_raw(:, 6));   % observer th1  estimate [rad]
xhat_dth1 = deg2rad(y_raw(:, 7));   % observer dth1 estimate [rad/s]
xhat_th2  = deg2rad(y_raw(:, 8));   % observer th2  estimate [rad]
xhat_dth2 = deg2rad(y_raw(:, 9));   % observer dth2 estimate [rad/s]

% Note: if observer outputs radians already, remove deg2rad()
% Check by comparing ranges:
fprintf('\nObserver state ranges:\n');
fprintf('  xhat_th1:  %.1f to %.1f\n', min(y_raw(:,6)), max(y_raw(:,6)));
fprintf('  xhat_dth1: %.1f to %.1f\n', min(y_raw(:,7)), max(y_raw(:,7)));
fprintf('  xhat_th2:  %.1f to %.1f\n', min(y_raw(:,8)), max(y_raw(:,8)));
fprintf('  xhat_dth2: %.1f to %.1f\n', min(y_raw(:,9)), max(y_raw(:,9)));

%% Trim
iS = round(trim_start/h_out) + 1;
iE = min(round(trim_end/h_out) + 1, length(t_sim));

t_plot     = t_sim(iS:iE);
u_plot     = u_sim(iS:iE);

%% Plot all 4 states: differentiator vs observer
figure('Name', 'Observer vs differentiator — all states');

% th1
subplot(4,1,1);
plot(t_plot, rad2deg(th1_rad(iS:iE)),  'b', 'LineWidth', 1.2); hold on;
plot(t_plot, rad2deg(xhat_th1(iS:iE)), 'r--', 'LineWidth', 1.2);
ylabel('th1 [deg]', 'Interpreter','none'); grid on;
legend('Encoder','Observer','Location','best','Interpreter','none');
title('Observer vs encoder — angles should match exactly');

% dth1
subplot(4,1,2);
plot(t_plot, rad2deg(dth1_rad(iS:iE)),  'b', 'LineWidth', 1.2); hold on;
plot(t_plot, rad2deg(xhat_dth1(iS:iE)), 'r--', 'LineWidth', 1.5);
ylabel('dth1 [deg/s]', 'Interpreter','none'); grid on;
legend('Differentiator','Observer','Location','best','Interpreter','none');
title('Velocity: observer should be smoother than differentiator');

% th2
subplot(4,1,3);
plot(t_plot, rad2deg(th2_rad(iS:iE)),  'b', 'LineWidth', 1.2); hold on;
plot(t_plot, rad2deg(xhat_th2(iS:iE)), 'r--', 'LineWidth', 1.2);
ylabel('th2 [deg]', 'Interpreter','none'); grid on;
legend('Encoder','Observer','Location','best','Interpreter','none');

% dth2
subplot(4,1,4);
plot(t_plot, rad2deg(dth2_rad(iS:iE)),  'b', 'LineWidth', 1.2); hold on;
plot(t_plot, rad2deg(xhat_dth2(iS:iE)), 'r--', 'LineWidth', 1.5);
ylabel('dth2 [deg/s]', 'Interpreter','none'); grid on;
legend('Differentiator','Observer','Location','best','Interpreter','none');
xlabel('Time [s]');

%% RMS errors
fprintf('\n--- Observer performance ---\n');
fprintf('th1  RMS error: %.3f deg\n',   rms(rad2deg(th1_rad(iS:iE)  - xhat_th1(iS:iE))));
fprintf('dth1 RMS error: %.2f deg/s\n', rms(rad2deg(dth1_rad(iS:iE) - xhat_dth1(iS:iE))));
fprintf('th2  RMS error: %.3f deg\n',   rms(rad2deg(th2_rad(iS:iE)  - xhat_th2(iS:iE))));
fprintf('dth2 RMS error: %.2f deg/s\n', rms(rad2deg(dth2_rad(iS:iE) - xhat_dth2(iS:iE))));

fprintf('\nWhat to look for:\n');
fprintf('  th1/th2 errors ~ 0      : angles tracked correctly\n');
fprintf('  dth1/dth2 smoother      : observer filtering noise\n');
fprintf('  dth errors < 20 deg/s   : good observer performance\n');
