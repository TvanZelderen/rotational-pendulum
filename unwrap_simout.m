function [th1, dth1, th2, dth2, psi] = unwrap_simout(simout, fc_Hz, h)
% Extract angle columns from simout with continuous, glitch-free th1.
%
% Strategy: unwrap raw encoder first, then filter over the dead-zone.
%   1. Detect dead-zone samples from raw y(:,1) — position-based,
%      revolution-count-independent.
%   2. Unwrap the raw signal first (dead-zone dip < 180° so unwrap ignores
%      it; real 360° wraps get resolved here).
%   3. NaN-out bad samples in unwrapped space, fill with linear bridge
%      (straight line from last good before to first good after — constant
%      slope inside the block, no spike in dth1 regardless of gap length).
%   4. Recompute dth1 from diff(th1)/h — derivative is constant inside the bridge.
%
% Tune THETA_DEAD, TOL, FILL_WIN if the patch misses or over-reaches.
%
% fc_Hz : low-pass cutoff for dth1 [Hz], default 5.
%         Set 0 for raw derivative (when caller applies its own filtfilt).
% h     : sample period [s]. If omitted, computed from simout.Time.

    THETA_DEAD = 82.1813;   % [deg] — confirmed dead-zone angle
    TOL        = 8;         % [deg] — half-window around dead-zone

    if nargin < 2 || isempty(fc_Hz);  fc_Hz = 5;                    end
    if nargin < 3 || isempty(h);      h     = mean(diff(simout.Time)); end

    y = simout.Data;

    % Step 1: detect dead-zone samples in raw (modular) signal
    th1_mod   = mod(y(:,1) - THETA_DEAD, 360);
    near_dead = th1_mod < TOL | th1_mod > 360 - TOL;
    % Extend each window by 1 to include the resume step
    bad = near_dead | [false; diff(near_dead(:)) < 0];

    % Step 2: unwrap raw first — dead-zone dip is < 180° so unwrap is
    % unaffected; real 360° wraps are resolved here.
    th1_unwrapped = unwrap(deg2rad(y(:,1))) * (180/pi);

    % Step 3: filter over the bad area in unwrapped space
    th1_clean = th1_unwrapped;
    th1_clean(bad) = NaN;
    th1 = fillmissing(th1_clean, 'linear', 'EndValues', 'extrap');

    % Step 4: recompute dth1 from the cleaned signal
    dth1_raw = [0; diff(th1)] / h;
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
