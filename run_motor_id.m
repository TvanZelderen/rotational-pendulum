% Motor characterisation — hardware only.  For simulation, see sim_motor_id.m.
% Prerequisites (once per session, in order):
%   calib.m  — opens fugiboard connection, resets encoder, activates relay
%   hwinit.m — sets sensor gain/offset calibration values
%
% Three run types (set run_type below):
%
%   'exploratory' — full-amplitude sine.  Set freq_hz and re-run for each frequency.
%                   Goal: read off dth1_max and see where arm 2 coupling kicks in.
%
%   'pre_id'      — low-amplitude, low-frequency sine (0.2 Hz, amp 0.3, 10 cycles).
%                   Goal: clean arm 1 motor response with arm 2 quiet.
%                   Saved with a labelled filename for sysid_driven.m.
%
%   'multisine'   — combination of sine waves, tuned with offset phases to stay within u-saturation.

%clear; clc;

pendulum_params;

run_type = 'pre_id';   % 'exploratory' | 'pre_id' | 'multisine'

h = 0.001;   % sample period [s] UPDATE THIS TO 0.001
run_time = 30; % to be used in simulink [s]

%% -----------------------------------------------------------------------
%  Run
% -----------------------------------------------------------------------
if strcmp(run_type, 'exploratory')

    freq_hz   = 1;
    amplitude = 0.3;
    cycles    = run_time / freq_hz;

    t  = (0 : h : run_time)';
    u  = amplitude * sin(2*pi*freq_hz*t);
    simin = [t, u];
    sim rotpentemplate;

    t_out = simout.Time;
    [th1, dth1, th2] = unwrap_simout(simout);

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
    cycles    = run_time / freq_hz;

    t  = (0 : h : run_time)';
    u  = amplitude * sin(2*pi*freq_hz*(t));
    simin = [t, u];
    sim rotpentemplate;

    t_out = simout.Time;
    [th1, dth1, th2] = unwrap_simout(simout);

    fprintf('Pre-ID | peak dth1 = %.1f deg/s | peak th2 = %.1f deg\n| peak ph1 = %.1f deg\n', ...
        max(abs(dth1)), max(abs(th2)), max(abs(th2+th1)));

    figure(2); clf;
    subplot(3,1,1); plot(t_out, th1);  ylabel('\theta_1 [deg]'); grid on;
    subplot(3,1,2); plot(t_out, dth1); ylabel('d\theta_1 [deg/s]'); grid on;
    subplot(3,1,3); plot(t_out, th2);  ylabel('\theta_2 [deg]'); grid on;
    xlabel('Time [s]');
    sgtitle(sprintf('Pre-ID — %.1f Hz, amp = %.1f', freq_hz, amplitude));
    save_run(simin, simout, sprintf('arm1_pre_id_f%03dmHz_a%02d', ...
        round(freq_hz*1000), round(amplitude*100)));

elseif strcmp(run_type, 'multisine')
    % Generate multisine input
    t    = (0 : h : run_time)';
    N    = length(t);
    df   = 1/run_time;

    % Frequency range around known natural frequency (1.691 Hz)
    f_min = 0.2;    % [Hz]
    f_max = 5.0;    % [Hz]
    freq_indices = round(f_min/df) : 2 : round(f_max/df);
    freqs = freq_indices * df;
    n_f   = length(freqs);

    % Schroeder phases
    k      = 1:n_f;
    phases = -pi * k.*(k-1) / n_f;

    % Build and scale
    u_raw = zeros(N, 1);
    for i = 1:n_f
        u_raw = u_raw + sin(2*pi*freqs(i)*t + phases(i));
    end
    amplitude = 0.8;   % START VERY SMALL — increase carefully
    u_ms = amplitude * u_raw / max(abs(u_raw));

    simin = [t, u_ms];

    fprintf('Multisine: %d freqs, %.2f–%.2f Hz, peak=%.3f\n', ...
            n_f, freqs(1), freqs(end), max(abs(u_ms)));

    sim rotpentemplate;

    t_out = simout.Time;
    [th1, dth1, th2] = unwrap_simout(simout);

    fprintf('Peak th1 = %.1f deg | peak dth1 = %.1f deg/s | peak th2 = %.1f deg\n', ...
            max(abs(th1)), max(abs(dth1)), max(abs(th2)));

    % Warn if rotating
    if max(abs(th1)) > 90
        warning('Arm 1 rotating! Reduce amplitude.');
    end
    
    figure(3); clf;
    subplot(3,1,1); plot(t_out, th1);  ylabel('\theta_1 [deg]'); grid on;
    subplot(3,1,2); plot(t_out, dth1); ylabel('d\theta_1 [deg/s]'); grid on;
    subplot(3,1,3); plot(t_out, th2);  ylabel('\theta_2 [deg]'); grid on;
    xlabel('Time [s]');
    sgtitle(sprintf('Multisine — %d freqs, amp=%.3f', n_f, amplitude));

    save_run(simin, simout, sprintf('multisine_amp%03d', round(amplitude*1000)));
    fprintf('Saved: multisine_amp%03d\n', round(amplitude*1000));
    
end
