% Collect arm-2 free-swing data (hardware only).
% After collecting, run sysid_freeswing.m for the offline parameter fit.
% Prerequisites (once per session, in order):
%   calib.m  — opens fugiboard connection, resets encoder, activates relay
%   hwinit.m — sets sensor gain/offset calibration values

clear; clc;
calib;
hwinit;

assert(exist('fugihandle', 'var'), ...
    'Run calib.m and hwinit.m before using hardware mode.');

h    = 0.01;   % sample period [s]
run_time = 30;     % experiment duration [s]
controller_sat = 0;

t     = (0:h:run_time)';
u     = zeros(size(t));
simin = [t, u];

sim rotpentemplate
save_run(simin, simout, 'link2_free_swing');
fprintf('Done. Run sysid_freeswing.m to fit parameters.\n');
