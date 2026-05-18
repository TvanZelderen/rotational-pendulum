% Offline arm-2 free-swing parameter fit.
% Collect data first with collect_freeswing.m.

clear; clc;

data_file = 'latest';   % 'latest' or exact filename, e.g. '20260511_170335_link2_free_swing.mat'

if strcmp(data_file, 'latest')
    files = dir(fullfile('data', '*link2_free_swing.mat'));
    [~, idx] = max([files.datenum]);
    data_file = files(idx).name;
end
load(fullfile('data', data_file), 'simin', 'simout');
fprintf('Loaded: %s\n', data_file);

h    = simin(2,1) - simin(1,1);
Tsim = simin(end,1);
t    = simin(:,1);

%% Extract phi = th1 + th2 (Simulink output 5), zero at equilibrium
y   = simout.Data;           % N x 5: [th1, dth1, th2, dth2, phi] in degrees
phi = y(:,5);
phi = mod(phi - mean(phi(end-100:end)) + 180, 360) - 180;

%% Overview plot — use to set trim_start / trim_end
figure;
plot(t, phi)
xlabel('Time [s]'); ylabel('\phi [deg]')

%% Trim end — cut noise-dominated tail
trim_end  = 18;    % [s] — stop before SNR collapses (~18 s from D4)
i_end     = numel(t) - (Tsim - trim_end) * 100;

%% Trim start — align to first natural peak so x0(2) = 0 is exact
% Search 100:500 (1–5 s) to skip the manual-release transient.
phi_search = phi(120:500);
[~, i_rel]  = max(abs(phi_search));
i_start     = 99 + i_rel;             % offset back to full-vector index

phi_trimmed = phi(i_start:i_end);
t_trimmed   = t(i_start:i_end);

%% Convert to radians (nonlinear ODE uses sin)
phi_rad = deg2rad(phi_trimmed);

%% idnlgrey — nonlinear grey-box identification
data = iddata(phi_rad, [], h);

alpha0 = 1;
beta0  = 9.81 / 0.1;
x0     = [phi_rad(1); 0];   % velocity = 0 at peak: physically exact

sys = idnlgrey('link2_ode', [1, 0, 2], {alpha0; beta0}, x0, 0);
sys.InitialStates(1).Fixed = false;
sys.InitialStates(2).Fixed = false;

opt = nlgreyestOptions('Display', 'on'); %WeightingFilter -> "bandpass [a, b]" -> "focus on estimation"
sys_est = nlgreyest(data, sys, opt);

fprintf('alpha (c2/m2/l2^2) = %.6f\n', sys_est.Parameters(1).Value);
fprintf('beta  (g/l2)       = %.6f\n', sys_est.Parameters(2).Value);
fprintf('l2                 = %.6f m\n', 9.81 / sys_est.Parameters(2).Value);

%% Compare measured vs model output
compare(data, sys_est);

%% ── DIAGNOSTICS ─────────────────────────────────────────────────────────────
% Run these cells after the fit to classify the dominant error source.
% Interpretation guide at the bottom of each cell.

%% D1 — Decay envelope shape  (exponential vs linear-in-cycle)
[pks, locs] = findpeaks(abs(phi_rad));
cycles = (1:numel(pks))';
figure('Name','D1 – Envelope');
subplot(1,2,1);
  plot(cycles, pks, 'o-');
  xlabel('Peak #'); ylabel('|\phi| [rad]'); title('Amplitude per peak');
subplot(1,2,2);
  semilogy(cycles, pks, 'o-');
  xlabel('Peak #'); ylabel('|\phi| [rad] (log)'); title('Log-scale — straight = viscous');
% Straight on log-y  → viscous damping dominates → fit problem is IC/trim/noise
% Concave (bends down) → Coulomb friction present → add -mu*sign(dth) to link2_ode

%% D2 — Arm-1 stationarity
figure('Name','D2 – Arm 1 motion');
subplot(2,1,1); plot(t, y(:,1)); ylabel('\theta_1 [deg]'); title('Arm 1 angle');
subplot(2,1,2); plot(t, y(:,2)); ylabel('\dot\theta_1 [deg/s]'); title('Arm 1 velocity');
dth2_max = max(abs(y(:,4)));
dth1_max = max(abs(y(:,2)));
fprintf('|dth1|_max / |dth2|_max = %.1f%%\n', 100 * dth1_max / dth2_max);
% >5%  → coupling torque leaks into arm-2 response; re-collect with arm 1 clamped

%% D3 — Period vs amplitude (nonlinear softening check)
zero_crossings = find(diff(sign(phi_rad)));
if numel(zero_crossings) >= 3
    half_periods = diff(t_trimmed(zero_crossings));   % successive zero-crossing gaps
    mid_idx      = zero_crossings(1:end-1);
    amp_at_cross = abs(phi_rad(mid_idx));
    figure('Name','D3 – Period vs amplitude');
    plot(amp_at_cross, half_periods * 2, 'o');
    xlabel('|\phi| at crossing [rad]'); ylabel('Period T [s]'); title('D3 – Period vs amplitude');
    yline(2*pi/sqrt(sys_est.Parameters(2).Value), '--', 'T_0 linear');
    % Upward slope = expected nonlinear lengthening; large scatter = noise/Coulomb
else
    fprintf('D3: too few zero crossings in trimmed window — widen trim.\n');
end

%% D4 — Trim sensitivity (tabulate alpha, beta, fit% vs trim window)
trim_starts = [1, 2, 3, 4];
trim_ends   = [18, 22, 26, 30];
fprintf('\n%-10s %-10s %-10s %-10s %-8s\n','trim_start','trim_end','alpha','beta','fit%');
for ts = trim_starts
    for te = trim_ends
        i_s = ts*100+1;  i_e = numel(t)-(Tsim-te)*100;
        if i_e <= i_s; continue; end
        phi_t = deg2rad(phi(i_s:i_e));
        t_t   = t(i_s:i_e);
        d_t   = iddata(phi_t,[],h);
        x0_t  = [phi_t(1); (phi_t(2)-phi_t(1))/h];
        s_t   = idnlgrey('link2_ode',[1,0,2],{sys_est.Parameters(1).Value; sys_est.Parameters(2).Value},x0_t,0);
        s_t.InitialStates(1).Fixed = false; s_t.InitialStates(2).Fixed = false;
        opt_t = nlgreyestOptions('Display','off');
        try
            s_t  = nlgreyest(d_t, s_t, opt_t);
            [~,fit_t] = compare(d_t, s_t);
            fprintf('%-10g %-10g %-10.4f %-10.4f %-8.1f\n', ts, te, ...
                s_t.Parameters(1).Value, s_t.Parameters(2).Value, fit_t);
        catch
            fprintf('%-10g %-10g FAILED\n', ts, te);
        end
    end
end
% If alpha or beta jumps >20% across trim choices → IC artefacts; fix x0 manually

%% D5 — Residual spectrum
phi_sim = sim(sys_est, data);
residual = phi_rad - phi_sim.OutputData;
figure('Name','D5 – Residual spectrum');
subplot(2,1,1);
  plot(t_trimmed, residual); xlabel('t [s]'); ylabel('residual [rad]'); title('D5 – Time residual');
subplot(2,1,2);
  pwelch(residual, [], [], [], 1/h);
  omega_n = sqrt(sys_est.Parameters(2).Value);
  xline(omega_n/(2*pi), '--', '\omega_n'); xline(2*omega_n/(2*pi), ':', '2\omega_n');
  xlabel('Frequency [Hz]'); title('Power spectrum of residual');
% Tone at 2*omega_n / (2*pi) Hz → Coulomb/quadratic signature
% Broadband flat                → measurement noise is the ceiling

%% ─────────────────────────────────────────────────────────────────────────────

%% Results

% Run 1 (2026-05-08): alpha = 0.325573, beta = 98.729513, l2 = 0.099362 m
% Run 2 (2026-05-08): alpha = 0.308600, beta = 98.646966, l2 = 0.099446 m
% Latest (2026-05-12): alpha = 0.164111, beta = 112.964563, l2 = 0.086841 m  ← locked into pendulum_params
