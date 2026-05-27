% Motor characterisation — terminal-velocity sweep, hardware only.
% Applies a staircase of constant u levels; arm 1 reaches terminal velocity
% at each step. Saves a single .mat file for sysid_termvel.m.
%
% Prerequisites (once per session):
%   calib.m  — opens fugiboard connection, resets encoder, activates relay
%   hwinit.m — sets sensor gain/offset calibration values

%clear; clc;

pendulum_params;

%% ── Config ───────────────────────────────────────────────────────────────────
h        = 0.01;   % sample period [s]
run_time = 50;     % [s] total — divided equally across all u levels

% u levels to sweep. Include both signs so tauc_kinetic is well-conditioned:
% positive u → positive omega_ss (sign = +1)
% negative u → negative omega_ss (sign = −1)
% Ascending positive then descending negative keeps direction changes minimal.
u_seq = [0.2, 0.4, 0.6, 0.8, 1, -1, -0.8, -0.6, -0.4, -0.2];

%% ── Build staircase input ────────────────────────────────────────────────────
n_steps = length(u_seq);
t_step  = run_time / n_steps;
t_total = run_time;

t = (0:h:t_total)';
u = zeros(size(t));

for k = 1:n_steps
    i0 = round((k-1)*t_step / h) + 1;
    i1 = min(round(k*t_step / h) + 1, length(t));
    u(i0:i1) = u_seq(k);
end

simin = [t, u];

%% ── Run ──────────────────────────────────────────────────────────────────────
sim rotpentemplate;

t_out = simout.Time;
y     = simout.Data;
dth1  = y(:, 2);   % [deg/s]

%% ── Quick-look plot ──────────────────────────────────────────────────────────
figure(1); clf;
subplot(2,1,1);
  stairs(t, u); ylabel('u [-]'); grid on; title('Terminal-velocity sweep');
subplot(2,1,2);
  plot(t_out, dth1); ylabel('d\theta_1 [deg/s]'); xlabel('Time [s]'); grid on;

fprintf('Peak |dth1| = %.1f deg/s\n', max(abs(dth1)));

fprintf('\n--- Terminal velocity per level ---\n');
fprintf('  %-6s  %-12s  %-12s\n', 'u', 'dth1 [deg/s]', 'dth1 [rad/s]');
for k = 1:n_steps
    i0 = round((k-1)*t_step / h) + 1;
    i1 = min(round(k*t_step / h) + 1, length(t_out));
    last_quarter = round(i0 + 0.75*(i1-i0)) : i1;
    dth1_ss = mean(dth1(last_quarter));
    fprintf('  %-6.2f  %-12.2f  %-12.4f\n', u_seq(k), dth1_ss, deg2rad(dth1_ss));
end

% --- Terminal velocity per level ---
%   u       dth1 [deg/s]  dth1 [rad/s]
%   0.20    -62.81        -1.0962     
%   0.40    -136.99       -2.3909     
%   0.60    -229.03       -3.9973     
%   0.80    275.26        4.8042      
%   1.00    -107.19       -1.8708     
%   -1.00   109.18        1.9055      
%   -0.80   19.20         0.3352      
%   -0.60   -69.14        -1.2066     
%   -0.40   -156.27       -2.7274     
%   -0.20   59.11         1.0316  

%% ── Save ─────────────────────────────────────────────────────────────────────
save_run(simin, simout, sprintf('arm1_termvel_%dsteps', n_steps));
