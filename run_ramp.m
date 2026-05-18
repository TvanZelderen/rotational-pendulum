% Motor characterisation — ramp test, hardware only.
% Identifies breakaway input at two arm-1 positions for km / tauc_static.
% See sysid_ramp.m for the offline fit.
%
% Prerequisites (once per session):
%   calib.m  — opens fugiboard connection, resets encoder, activates relay
%   hwinit.m — sets sensor gain/offset calibration values
%
% Procedure:
%   1. Position arm 1 at theta1 = 0 deg (hanging straight down). Run Cell A.
%   2. Position arm 1 at theta1 = 90 deg (horizontal). Run Cell B.
%   Each cell saves its own .mat file for sysid_ramp.m.

%clear; clc;

pendulum_params;

%% ── Shared config ────────────────────────────────────────────────────────────
h         = 0.01;    % sample period [s]
amplitude = 0.1;     % ramp rate [u/s] — u reaches amplitude*run_time at end
run_time  = 5;       % [s] — keep short; breakaway expected well before u = amplitude*run_time

%% ── Cell A: theta1 = 0 deg (arm hanging straight down) ──────────────────────
% Position arm 1 at 0 degrees before running this cell.

theta1_label = 0;   % [deg] — label only; used in filename

t = (0:h:run_time)';
u = amplitude * t;
simin = [t, u];
sim rotpentemplate;

t_out = simout.Time;
[th1_A, dth1_A] = unwrap_simout(simout);

figure(1); clf;
subplot(2,1,1); plot(t_out, th1_A);  ylabel('\theta_1 [deg]'); grid on;
subplot(2,1,2); plot(t_out, dth1_A); ylabel('d\theta_1 [deg/s]'); grid on;
xlabel('Time [s]');
sgtitle(sprintf('Ramp — \\theta_1 = %d deg, rate = %.2f u/s', theta1_label, amplitude));

save_run(simin, simout, sprintf('arm1_ramp_th%03d_a%02d', theta1_label, round(amplitude*100)));

%% ── Cell B: theta1 = 90 deg (arm horizontal) ────────────────────────────────
% Reposition arm 1 to 90 degrees before running this cell.

theta1_label = 90;   % [deg]

t = (0:h:run_time)';
u = amplitude * t;
simin = [t, u];
sim rotpentemplate;

t_out = simout.Time;
[th1_B, dth1_B] = unwrap_simout(simout);

figure(2); clf;
subplot(2,1,1); plot(t_out, th1_B);  ylabel('\theta_1 [deg]'); grid on;
subplot(2,1,2); plot(t_out, dth1_B); ylabel('d\theta_1 [deg/s]'); grid on;
xlabel('Time [s]');
sgtitle(sprintf('Ramp — \\theta_1 = %d deg, rate = %.2f u/s', theta1_label, amplitude));

save_run(simin, simout, sprintf('arm1_ramp_th%03d_a%02d', theta1_label, round(amplitude*100)));
