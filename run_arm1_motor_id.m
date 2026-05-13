% Motor characterisation — runs before the full arm 1 ID experiment.
%
% Two run types (set run_type below):
%
%   'exploratory' — full-amplitude sine at several frequencies.
%                   Goal: read off dth1_max and see where arm 2 coupling kicks in.
%                   In simulation: loops automatically over test_freqs.
%                   On hardware:   set freq_hz manually and re-run for each frequency.
%
%   'pre_id'      — low-amplitude, low-frequency sine.
%                   Goal: clean arm 1 motor response with arm 2 quiet.

clear; clc;

%% -----------------------------------------------------------------------
%  MODE SELECT
% -----------------------------------------------------------------------
run_with_simulation = false;   % true → simulation | false → real hardware
run_type = 'exploratory';     % 'exploratory' | 'pre_id'
run_label = run_type;         % used for saved filename; override if needed

%% -----------------------------------------------------------------------
%  Init
% -----------------------------------------------------------------------
if run_with_simulation
    pendulum_params;
else
    assert(exist('fugihandle', 'var'), ...
        'Run calib.m and hwinit.m before using hardware mode.');
end

h = 0.01;   % sample period [s]

%% -----------------------------------------------------------------------
%  Run
% -----------------------------------------------------------------------
if strcmp(run_type, 'exploratory')

    amplitude  = 1.0;
    cycles     = 5;
    test_freqs = [0.2, 0.5, 1.0, 2.0];   % [Hz]
    freq_hz    = 0.5;   % hardware only: change between runs

    if run_with_simulation
        peak_dth1 = zeros(size(test_freqs));
        peak_th2  = zeros(size(test_freqs));

        for i = 1:length(test_freqs)
            f  = test_freqs(i);
            t  = (0 : h : cycles/f)';
            u  = amplitude * sin(2*pi*f*t);
            simin = [t, u];
            run_sim;
            y = simout.Data;
            peak_dth1(i) = max(abs(y(:,2)));
            peak_th2(i)  = max(abs(y(:,3)));
        end

        figure(1); clf;
        yyaxis left
        plot(test_freqs, peak_dth1, 'o-');
        ylabel('Peak |d\theta_1| [deg/s]');
        yyaxis right
        plot(test_freqs, peak_th2, 's--');
        ylabel('Peak |\theta_2| [deg]');
        xlabel('Frequency [Hz]');
        legend('dth1 peak', 'th2 peak');
        title('Exploratory: arm 1 velocity and arm 2 coupling vs frequency');
        grid on;

        fprintf('\nPeak dth1 by frequency:\n');
        for i = 1:length(test_freqs)
            fprintf('  %.1f Hz -> %.1f deg/s  (peak th2 = %.1f deg)\n', ...
                test_freqs(i), peak_dth1(i), peak_th2(i));
        end

    else
        t  = (0 : h : cycles/freq_hz)';
        u  = amplitude * sin(2*pi*freq_hz*t);
        simin = [t, u];
        sim rotpentemplate;
        y    = simout.Data;
        th1  = mod(y(:,1) + 180, 360) - 180;
        dth1 = y(:,2);
        th2  = mod(y(:,3) + 180, 360) - 180;

        fprintf('freq = %.1f Hz | peak dth1 = %.1f deg/s | peak th2 = %.1f deg\n', ...
            freq_hz, max(abs(dth1)), max(abs(th2)));

        figure(1); clf;
        subplot(3,1,1); plot(t, th1);  ylabel('\theta_1 [deg]'); grid on;
        subplot(3,1,2); plot(t, dth1); ylabel('d\theta_1 [deg/s]'); grid on;
        subplot(3,1,3); plot(t, th2);  ylabel('\theta_2 [deg]'); grid on;
        xlabel('Time [s]');
        sgtitle(sprintf('Exploratory — %.1f Hz, amp = %.1f', freq_hz, amplitude));
        save_run(simin, simout, run_label);
    end

elseif strcmp(run_type, 'pre_id')

    freq_hz   = 0.2;
    amplitude = 0.3;
    cycles    = 10;

    t  = (0 : h : cycles/freq_hz)';
    u  = amplitude * sin(2*pi*freq_hz*t);
    simin = [t, u];

    if run_with_simulation
        run_sim;
    else
        sim rotpentemplate;
    end

    y    = simout.Data;
    th1  = mod(y(:,1) + 180, 360) - 180;
    dth1 = y(:,2);
    th2  = mod(y(:,3) + 180, 360) - 180;

    fprintf('Pre-ID | peak dth1 = %.1f deg/s | peak th2 = %.1f deg\n', ...
        max(abs(dth1)), max(abs(th2)));

    figure(2); clf;
    subplot(3,1,1); plot(t, th1);  ylabel('\theta_1 [deg]'); grid on;
    subplot(3,1,2); plot(t, dth1); ylabel('d\theta_1 [deg/s]'); grid on;
    subplot(3,1,3); plot(t, th2);  ylabel('\theta_2 [deg]'); grid on;
    xlabel('Time [s]');
    sgtitle(sprintf('Pre-ID — %.1f Hz, amp = %.1f', freq_hz, amplitude));
    save_run(simin, simout, run_label);

end
