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
h      = 0.01;   % sample period [s]
t_step = 5;      % [s] per level — long enough for arm 1 to reach terminal velocity

% u levels to sweep. Include both signs so tauc_kinetic is well-conditioned:
% positive u → positive omega_ss (sign = +1)
% negative u → negative omega_ss (sign = −1)
% Ascending positive then descending negative keeps direction changes minimal.
u_seq = [0.2, 0.4, 0.6, 0.8, 1, -1, -0.8, -0.6, -0.4, -0.2];

%% ── Build staircase input ────────────────────────────────────────────────────
n_steps = length(u_seq);
t_total = n_steps * t_step;

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

%% ── Save ─────────────────────────────────────────────────────────────────────
save_run(simin, simout, sprintf('arm1_termvel_%dsteps', n_steps));
