function [th1, dth1, th2, dth2, psi] = unwrap_simout(simout, fc_Hz, h)
% Extract angle columns from simout with continuous, glitch-free th1.
%
% Strategy: unwrap raw encoder first, then extrapolate over the dead-zone.
%   1. Detect dead-zone samples from raw y(:,1) — position-based,
%      revolution-count-independent. Dilate mask ±3 samples.
%   2. Unwrap the raw signal first (dead-zone dip < 180° so unwrap ignores
%      it; real 360° wraps are resolved here).
%   3. NaN-out bad samples in unwrapped space. Fill each window by
%      forward-extrapolating at constant velocity from the last good sample
%      before the window. This avoids using the post-crossing encoder
%      position, which has an offset due to the dead-zone counter reset.
%   4. Recompute dth1 from diff(th1)/h — derivative is constant inside bridge.
%
% Tune THETA_DEAD, TOL if the patch misses or over-reaches.
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
    % Dilate by 3 samples each direction — covers transition artifacts at bridge edges
    se  = ones(7, 1);
    bad = conv(double(near_dead), se, 'same') > 0;

    % Step 2: unwrap raw first — dead-zone dip is < 180° so unwrap is
    % unaffected; real 360° wraps are resolved here.
    th1_unwrapped = unwrap(deg2rad(y(:,1))) * (180/pi);

    % Step 3: forward-extrapolate over each bad window at pre-crossing velocity,
    % then correct the encoder offset at the exit so the resume is seamless.
    % Using the post-crossing encoder position directly introduces a slope
    % error (wrong bridge velocity) and a spike at the exit (position jump)
    % because the encoder counter resets inside the dead zone.
    th1 = th1_unwrapped;
    bad_idx = find(bad(:)');
    if ~isempty(bad_idx)
        win_starts = bad_idx([true,  diff(bad_idx) > 1]);
        win_ends   = bad_idx([diff(bad_idx) > 1, true]);
        for wi = 1:numel(win_starts)
            i0 = win_starts(wi);
            i1 = win_ends(wi);
            vel = 0;
            if i0 > 2
                vel = th1(i0-1) - th1(i0-2);   % [deg/sample] from corrected signal
            end
            % Fill bad window with constant-velocity extrapolation
            for k = i0:i1
                th1(k) = th1(i0-1) + vel * (k - i0 + 1);
            end
            % At exit: shift all remaining samples by the encoder offset so
            % the resume joins seamlessly onto the extrapolation.
            i_resume = i1 + 1;
            if i_resume <= length(th1)
                extrap_at_resume = th1(i0-1) + vel * (i_resume - i0 + 1);
                offset = th1(i_resume) - extrap_at_resume;
                th1(i_resume:end) = th1(i_resume:end) - offset;
            end
        end
    end

    % Step 4: recompute dth1 from the cleaned signal
    dth1_raw = [0; diff(th1)] / h;
    if fc_Hz > 0
        fs = 1/h;
        [b_f, a_f] = butter(2, fc_Hz / (fs/2));
        dth1 = filtfilt(b_f, a_f, medfilt1(dth1_raw, 7));
    else
        dth1 = dth1_raw;
    end

    th2      = unwrap(deg2rad(y(:,3))) * (180/pi);
    dth2_raw = [0; diff(th2)] / h;
    dth2_raw(abs([0; diff(th2)]) > 90) = NaN;   % kill wrap spikes (1-2 samples per crossing)
    dth2_raw = fillmissing(dth2_raw, 'linear');
    if fc_Hz > 0
        dth2 = filtfilt(b_f, a_f, dth2_raw);
    else
        dth2 = dth2_raw;
    end
    psi  = th1 + th2;
end
