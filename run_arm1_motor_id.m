% Motor characterisation — hardware only.  For simulation, see sim_arm1_motor_id.m.
% Prerequisites (once per session, in order):
%   calib.m  — opens fugiboard connection, resets encoder, activates relay
%   hwinit.m — sets sensor gain/offset calibration values
%
% Two run types (set run_type below):
%
%   'exploratory' — full-amplitude sine.  Set freq_hz and re-run for each frequency.
%                   Goal: read off dth1_max and see where arm 2 coupling kicks in.
%
%   'pre_id'      — low-amplitude, low-frequency sine (0.2 Hz, amp 0.3, 10 cycles).
%                   Goal: clean arm 1 motor response with arm 2 quiet.
%                   Saved with a labelled filename for sysid_arm1_driven.m.

%clear; clc;

pendulum_params;
assert(exist('fugihandle', 'var'), ...
    'Run calib.m and hwinit.m before using hardware mode.');

run_type = 'pre_id';   % 'exploratory' | 'pre_id'

h = 0.01;   % sample period [s]

%% -----------------------------------------------------------------------
%  Run
% -----------------------------------------------------------------------
if strcmp(run_type, 'exploratory')

    freq_hz   = 1;
    amplitude = 0.3;
    cycles    = 30*freq_hz; % set this, then re-run for each frequency

    t  = (0 : h : cycles/freq_hz)';
    u  = amplitude * sin(2*pi*freq_hz*t);
    simin = [t, u];
    sim rotpentemplate;

    t_out = simout.Time;
    y    = simout.Data;
    th1  = mod(y(:,1) + 180, 360) - 180;
    dth1 = y(:,2);
    th2  = mod(y(:,3) + 180, 360) - 180;

    fprintf('freq = %.1f Hz | peak dth1 = %.1f deg/s | peak th2 = %.1f deg\n', ...
        freq_hz, max(abs(dth1)), max(abs(th2)));

    figure(1); clf;
    subplot(3,1,1); plot(t_out, th1);  ylabel('\theta_1 [deg]'); grid on;
    subplot(3,1,2); plot(t_out, dth1); ylabel('d\theta_1 [deg/s]'); grid on;
    subplot(3,1,3); plot(t_out, th2);  ylabel('\theta_2 [deg]'); grid on;
    xlabel('Time [s]');
    sgtitle(sprintf('Exploratory — %.1f Hz, amp = %.1f', freq_hz, amplitude));
    save_run(simin, simout, sprintf('arm1_exploratory_f%03dmHz', round(freq_hz*1000)));

elseif strcmp(run_type, 'pre_id')

    freq_hz   = 0.3;
    amplitude = 0.3;
    cycles    = 30*freq_hz;

    t  = (0 : h : cycles/freq_hz)';
    u  = amplitude * sin(2*pi*freq_hz*(t));
    simin = [t, u];
    sim rotpentemplate;

    y    = simout.Data;
    th1  = mod(y(:,1) + 180, 360) - 180;
    dth1 = y(:,2);
    th2  = mod(y(:,3) + 180, 360) - 180;

    fprintf('Pre-ID | peak dth1 = %.1f deg/s | peak th2 = %.1f deg\n| peak ph1 = %.1f deg\n', ...
        max(abs(dth1)), max(abs(th2)), max(abs(th2+th1)));

    figure(2); clf;
    subplot(3,1,1); plot(t, th1);  ylabel('\theta_1 [deg]'); grid on;
    subplot(3,1,2); plot(t, dth1); ylabel('d\theta_1 [deg/s]'); grid on;
    subplot(3,1,3); plot(t, th2);  ylabel('\theta_2 [deg]'); grid on;
    xlabel('Time [s]');
    sgtitle(sprintf('Pre-ID — %.1f Hz, amp = %.1f', freq_hz, amplitude));
    save_run(simin, simout, sprintf('arm1_pre_id_f%03dmHz_a%02d', ...
        round(freq_hz*1000), round(amplitude*100)));

end
