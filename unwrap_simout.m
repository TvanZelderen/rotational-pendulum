function [th1, dth1, th2, dth2, psi] = unwrap_simout(simout, fc_Hz, h)
% Extract angle columns from simout with continuous, glitch-free th1.
%
% Strategy: unwrap raw encoder first, then extrapolate over the dead-zone.
%   1. Detect dead-zone samples from raw y(:,1) — position-based,
%      revolution-count-independent. Dilate mask ±3 samples.
%   2. Unwrap the raw signal first (dead-zone dip < 180° so unwrap ignores
%      it; real 360° wraps are resolved here).
%   3. Two-sided LS bridge over each bad window.
%      Joint model: pre-samples follow  th1[s] = a + v*s,
%                   post-samples follow th1[s] = a + v*s + delta
%      where delta is the encoder-reset offset.  Unknowns [a, v, delta]
%      solved in LS from N_FIT good samples each side.  Bridge filled with
%      the no-offset line; tail shifted by -delta so the post region joins
%      seamlessly.  Avoids single-sample fragility of the old vel/offset
%      estimates and removes the offset-induced exit spike.
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

    % Step 3: two-sided LS bridge.
    N_FIT = 7;   % good samples each side used for the joint fit

    th1 = th1_unwrapped;
    bad_idx = find(bad(:)');
    if ~isempty(bad_idx)
        win_starts = bad_idx([true,  diff(bad_idx) > 1]);
        win_ends   = bad_idx([diff(bad_idx) > 1, true]);

        for wi = 1:numel(win_starts)
            i0 = win_starts(wi);
            i1 = win_ends(wi);
            ir = i1 + 1;   % first post-window sample index

            % --- collect good pre-window indices (backwards from i0-1) ---
            pre_idx = zeros(1, N_FIT);  n_pre = 0;
            k = i0 - 1;
            while k >= 1 && n_pre < N_FIT
                if ~bad(k);  n_pre = n_pre + 1;  pre_idx(n_pre) = k;  end
                k = k - 1;
            end
            pre_idx = fliplr(pre_idx(1:n_pre));   % ascending order

            if n_pre == 0;  continue;  end   % no pre samples — skip

            % --- collect good post-window indices (forwards from ir) ---
            post_idx = zeros(1, N_FIT);  n_post = 0;
            k = ir;
            while k <= length(th1) && n_post < N_FIT
                if ~bad(k);  n_post = n_post + 1;  post_idx(n_post) = k;  end
                k = k + 1;
            end
            post_idx = post_idx(1:n_post);

            % Centre sample indices for numerical conditioning
            s_all    = [pre_idx(:); post_idx(:)];
            s_c      = mean(s_all);
            s_norm   = s_all - s_c;

            if n_post == 0
                % No post samples — extrapolate pre-side only, no tail shift
                if n_pre >= 2
                    p = polyfit(pre_idx(:) - s_c, th1(pre_idx(:)), 1);
                else
                    p = [0, th1(pre_idx(end))];
                end
                for kf = i0:i1
                    th1(kf) = p(1)*(kf - s_c) + p(2);
                end
                continue;
            end

            % Joint LS: A * [a; v; delta] = b
            %   pre  rows: [1,  s_norm,  0]
            %   post rows: [1,  s_norm,  1]
            A_ls  = [ones(n_pre+n_post, 1), ...
                     s_norm, ...
                     [zeros(n_pre, 1); ones(n_post, 1)]];
            b_ls  = th1([pre_idx(:); post_idx(:)]);
            coef  = A_ls \ b_ls;   % [a;  v;  delta]
            a_fit = coef(1);
            v_fit = coef(2);
            delta = coef(3);

            % Fill bridge with no-offset line
            for kf = i0:i1
                th1(kf) = a_fit + v_fit*(kf - s_c);
            end

            % Shift all remaining samples by -delta so post region joins bridge
            if ir <= length(th1)
                th1(ir:end) = th1(ir:end) - delta;
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
