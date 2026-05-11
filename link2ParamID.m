clear; clc;
% Prerequisites (run once per session before this script):
%   1. calib.m  — opens fugiboard connection, resets encoder, activates relay
%   2. hwinit.m — sets sensor gain/offset calibration values

%% Parameters
h    = 0.01;  % sample period [s]
Tsim = 30;    % experiment duration [s]

%% Input signal — zero to hold arm 1 stationary
t     = (0:h:Tsim)';
u     = zeros(size(t));
simin = [t, u];

%% Run experiment
sim rotpentemplate
save_run(simin, simout, 'link2_free_swing');

%% Extract phi = th1 + th2 (Simulink output 5), zero at equilibrium
y   = simout.Data;           % N x 5: [th1, dth1, th2, dth2, phi] in degrees
phi = y(:,5);
phi = mod(phi - mean(phi(end-100:end)) + 180, 360) - 180;

%% Overview plot — use to set trimStart / trimEnd
figure;
plot(t, phi)
xlabel('Time [s]'); ylabel('\phi [deg]')

%% Trim end — cut noise-dominated tail
trimEnd  = 18;    % [s] — stop before SNR collapses (~18 s from D4)
iEnd     = numel(t) - (Tsim - trimEnd) * 100;

%% Trim start — align to first natural peak so x0(2) = 0 is exact
% Search 100:500 (1–5 s) to skip the manual-release transient.
phi_search = phi(100:500);
[~, iRel]  = max(abs(phi_search));
iStart     = 99 + iRel;             % offset back to full-vector index

phi_trimmed = phi(iStart:iEnd);
tTrimmed    = t(iStart:iEnd);

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

opt = nlgreyestOptions('Display', 'on');
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
    half_periods = diff(tTrimmed(zero_crossings));   % successive zero-crossing gaps
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
trimStarts = [1, 2, 3, 4];
trimEnds   = [18, 22, 26, 30];
fprintf('\n%-10s %-10s %-10s %-10s %-8s\n','trimStart','trimEnd','alpha','beta','fit%');
for ts = trimStarts
    for te = trimEnds
        iS = ts*100+1;  iE = numel(t)-(Tsim-te)*100;
        if iE <= iS; continue; end
        phi_t = deg2rad(phi(iS:iE));
        t_t   = t(iS:iE);
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
  plot(tTrimmed, residual); xlabel('t [s]'); ylabel('residual [rad]'); title('D5 – Time residual');
subplot(2,1,2);
  pwelch(residual, [], [], [], 1/h);
  omega_n = sqrt(sys_est.Parameters(2).Value);
  xline(omega_n/(2*pi), '--', '\omega_n'); xline(2*omega_n/(2*pi), ':', '2\omega_n');
  xlabel('Frequency [Hz]'); title('Power spectrum of residual');
% Tone at 2*omega_n / (2*pi) Hz → Coulomb/quadratic signature
% Broadband flat                → measurement noise is the ceiling

%% ─────────────────────────────────────────────────────────────────────────────

%% Results

% alpha (c2/m2/l2^2) = 0.325573
% beta  (g/l2)       = 98.729513
% l2                 = 0.099362 m

% alpha (c2/m2/l2^2) = 0.308600
% beta  (g/l2)       = 98.646966
% l2                 = 0.099446 m