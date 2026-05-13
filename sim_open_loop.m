% Simulation open-loop run.  For hardware, see run_open_loop.m.

clear; clc;

pendulum_params;

save_sim  = false;        % set true to write simout to data/
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
run_sim;

%% -----------------------------------------------------------------------
%  Extract outputs
% -----------------------------------------------------------------------
[th1, dth1, th2, dth2, psi] = wrap_simout(simout);

%% Save (off by default — flip save_sim to save)
if save_sim
    save_run(simin, simout, run_label);
end

%% Plot
figure(1); clf;
plot(t, th1, t, th2, t, psi);
legend('\theta_1', '\theta_2', '\psi');
xlabel('Time [s]'); ylabel('Angle [deg]'); grid on;
title('Open loop — simulation');
