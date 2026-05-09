% Open-loop experiment — works on real hardware or simulation.
% Toggle the flag below; everything else is identical.


clear; clc;

%% -----------------------------------------------------------------------
%  MODE SELECT
% -----------------------------------------------------------------------
run_with_simulation = true;   % true  → simulation (no hardware needed)
                               % false → real hardware (run calib.m + hwinit.m first)
run_label = 'open_loop';      % short label for saved data file

%% -----------------------------------------------------------------------
%  Init
% -----------------------------------------------------------------------

if run_with_simulation
    pendulum_params;           % loads struct p into workspace
else
    % Prerequisites (run once per session, in order):
    %   calib.m  — opens fugiboard connection, resets encoder, activates relay
    %   hwinit.m — sets sensor gain/offset calibration values
    % (They are not called here because calib.m resets the encoder;
    %  run them manually before starting a hardware session.)
    assert(exist('fugihandle', 'var'), ...
        'Run calib.m and hwinit.m before using hardware mode.');
end

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

simin = [t, u];   % col 1: time [s],  col 2: voltage command [V]

%% -----------------------------------------------------------------------
%  Run  — the only line that differs between modes
% -----------------------------------------------------------------------
if run_with_simulation
    run_sim;               % integrates ODE → sets simout in workspace
else
    sim rotpentemplate;    % runs on hardware → sets simout in workspace
end

%% -----------------------------------------------------------------------
%  Extract outputs
% -----------------------------------------------------------------------
y = simout.Data;   % N×5 matrix: [th1_deg, dth1_deg/s, th2_deg, dth2_deg/s, psi_deg]

%% Wrap angles to (-180, 180]
th1  = mod(y(:,1) + 180, 360) - 180;
dth1 = y(:,2);
th2  = mod(y(:,3) + 180, 360) - 180;
dth2 = y(:,4);
psi  = mod(y(:,5) + 180, 360) - 180;

% %% Steady-state statistics  (trim first and last second)
% th1_ss = th1(101:end-100);
% th2_ss = th2(101:end-100);
% 
% theta1_mean  = mean(th1_ss)
% theta2_mean  = mean(th2_ss)
% theta1_range = max(th1_ss) - min(th1_ss)
% theta2_range = max(th2_ss) - min(th2_ss)

%% Save
save_run(simin, simout, run_label);

%% Plot
figure(1); clf;
plot(t, th1, t, th2, t, psi);
legend('\theta_1', '\theta_2', '\psi');
xlabel('Time [s]'); ylabel('Angle [deg]'); grid on;
title('Open loop — rotational pendulum');
