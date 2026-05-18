function [th1, dth1, th2, dth2, psi] = unwrap_simout(simout, fc_Hz, h)
% Extract angle columns from simout with continuous th1.
% Unwraps th1 so encoder dead-zone jumps (at any physical angle) are removed.
% dth1 is recomputed from diff(th1)/h rather than taken from simout column 2.
% th2, dth2, psi pass through unchanged.
%
% fc_Hz : low-pass cutoff for dth1 [Hz], default 5. Set 0 for raw derivative
%         (use when the caller applies its own filtfilt).
% h     : sample period [s]. If omitted, computed from simout.Time.
%
% Signature mirrors wrap_simout — drop-in replacement in run_* scripts.

    if nargin < 2 || isempty(fc_Hz);  fc_Hz = 5;                    end
    if nargin < 3 || isempty(h);      h     = mean(diff(simout.Time)); end

    y = simout.Data;

    th1 = unwrap(deg2rad(y(:,1))) * (180/pi);   % continuous [deg]

    dth1_raw = [0; diff(th1)] / h;              % [deg/s], first sample zeroed
    if fc_Hz > 0
        fs = 1/h;
        [b_f, a_f] = butter(2, fc_Hz / (fs/2));
        dth1 = filtfilt(b_f, a_f, dth1_raw);
    else
        dth1 = dth1_raw;
    end

    th2  = y(:,3);
    dth2 = y(:,4);
    psi  = th1 + th2;
end
