% Arm-1 step-response identification run — hardware only.
% Pattern: 0 → u_level → 0 → next_level → 0 → ...
% Each rise from zero gives a clean first-order response for J1 estimation.
% Load output into sysid_step_id.m to extract time constants.
%
% Prerequisites (once per session):
%   calib.m  — opens fugiboard connection, resets encoder, activates relay
%   hwinit.m — sets sensor gain/offset calibration values

%clear; clc;

pendulum_params;

%% ── Config ───────────────────────────────────────────────────────────────────
h          = 0.01;   % sample period [s]
t_zero     = 5;      % [s] dwell at u=0 between steps (arm stops)
t_step     = 5;     % [s] dwell at each u level (arm reaches steady state)

u_levels = [0.2, 0.4, 0.6, 0.8, 1.0];

%% ── Build input ──────────────────────────────────────────────────────────────
% Pattern per level: [zeros for t_zero, u_level for t_step]
% Ends with a final zero segment.
n_zero = round(t_zero / h);
n_step = round(t_step / h);

u_vec = [];
for k = 1:length(u_levels)
    u_vec = [u_vec; zeros(n_zero, 1); u_levels(k) * ones(n_step, 1)]; %#ok<AGROW>
end
u_vec = [u_vec; zeros(n_zero, 1)];   % trailing zero

n_total  = length(u_vec);
t        = (0 : h : (n_total-1)*h)';
run_time = t(end);   % picked up by rotpentemplate StopTime
simin    = [t, u_vec];

%% ── Run ──────────────────────────────────────────────────────────────────────
sim rotpentemplate;

%% ── Quick-look plot ──────────────────────────────────────────────────────────
figure(1); clf;
subplot(2,1,1);
  stairs(t, u_vec); ylabel('u [-]'); grid on; title('Step-response ID sweep');
subplot(2,1,2);
  plot(simout.Time, simout.Data(:,2)); ylabel('d\theta_1 [deg/s]'); xlabel('Time [s]'); grid on;

%% ── Save ─────────────────────────────────────────────────────────────────────
save_run(simin, simout, sprintf('arm1_stepid_%dlevels', length(u_levels)));
