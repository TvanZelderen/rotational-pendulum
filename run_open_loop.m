% Open-loop hardware run.  For simulation, see sim_open_loop.m.
% Prerequisites (once per session, in order):
%   calib.m  — opens fugiboard connection, resets encoder, activates relay
%   hwinit.m — sets sensor gain/offset calibration values

clear; clc;

pendulum_params;
assert(exist('fugihandle', 'var'), ...
    'Run calib.m and hwinit.m before using hardware mode.');

run_label = 'open_loop';

%% -----------------------------------------------------------------------
%  Parameters
% -----------------------------------------------------------------------
h    = 0.01;   % sample period [s]
Tsim = 30;     % experiment duration [s]

%% -----------------------------------------------------------------------
%  Input signal
% -----------------------------------------------------------------------
t = (0 : h : Tsim)';

amplitude = 0.5;
omega     = 0.1;
u = amplitude * sin(omega * t) * 0;   % zero input (open loop)

simin = [t, u];   % col 1: time [s],  col 2: normalised motor command [-]

%% -----------------------------------------------------------------------
%  Run
% -----------------------------------------------------------------------
sim rotpentemplate;

%% -----------------------------------------------------------------------
%  Extract outputs
% -----------------------------------------------------------------------
[th1, dth1, th2, dth2, psi] = unwrap_simout(simout);

%% Save
save_run(simin, simout, run_label);

%% Plot
figure(1); clf;
plot(t, th1, t, th2, t, psi);
legend('\theta_1', '\theta_2', '\psi');
xlabel('Time [s]'); ylabel('Angle [deg]'); grid on;
title('Open loop — rotational pendulum');
