% Collect arm-2 free-swing data (hardware only).
% After collecting, run sysid_freeswing.m for the offline parameter fit.
% Prerequisites (once per session, in order):
%   calib.m  — opens fugiboard connection, resets encoder, activates relay
%   hwinit.m — sets sensor gain/offset calibration values

clear; clc;

assert(exist('fugihandle', 'var'), ...
    'Run calib.m and hwinit.m before using hardware mode.');

h    = 0.01;   % sample period [s]
Tsim = 30;     % experiment duration [s]

t     = (0:h:Tsim)';
u     = zeros(size(t));
simin = [t, u];

sim rotpentemplate
save_run(simin, simout, 'link2_free_swing');
fprintf('Done. Run sysid_freeswing.m to fit parameters.\n');
