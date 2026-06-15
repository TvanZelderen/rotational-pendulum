%% run_ref_tracking.m  —  LQI reference-tracking on hardware (down-up)
%
% Hardware counterpart to sim_ref_tracking.m.
% Arm 1 tracks a time-varying setpoint; pendulum stays upright.
%
% DEMO  0  hold        — arm holds θ₁ = 0  (first powered check)
% DEMO  1  step        — arm jumps +10° at t = 3 s
% DEMO  2  sine        — arm sweeps ±15° at 0.25 Hz
% DEMO  3  waypoints   — arm visits +20°, then −20°, then returns to 0°
%
% Bring-up order (read before first powered run):
%   1. DEMO 0, relay off / arm held — confirm logging, u stays bounded.
%   2. DEMO 0, powered, pendulum balanced by hand — confirm balance holds.
%   3. DEMO 1 (step +10°) — small move; watch u, watch θ₂ near 180°.
%   4. DEMO 2, 3. Raise amplitude only after each clean run.

%% -- Preamble --------------------------------------------

clear; clc;
calib;
hwinit;
pendulum_params;

%% -- Config ----------------------------------------------

DEMO = 3;             % 0 = hold | 1 = step | 2 = sine | 3 = waypoints

ref_eq = [0; 0; pi; 0];  % down-up equilibrium: arm 1 at 0, pendulum up
R      = 25;
h      = 0.001;           % sample period [s]
run_time = 30;

% These keep any non-tracking blocks in the model happy:
ref    = ref_eq;
offset = [0; -pi];
controller_sat = 1;

%% -- Reference trajectory --------------------------------

switch DEMO
    case 0
        run_time   = 8;
        ref_fn = @(t) 0;
        ttl    = 'Hold: θ₁ = 0°';
    case 1
        run_time   = 12;
        ref_fn = @(t) deg2rad(30) * (t >= 3);
        ttl    = 'Step: arm 1 +30° at t = 3 s';
    case 2
        run_time   = 12;
        ref_fn = @(t) deg2rad(45) * sin(2*pi * 0.25 * t);
        ttl    = 'Sine sweep: ±15°, 0.25 Hz';
    case 3
        run_time   = 13;
        ref_fn = @(t) deg2rad(35)*(t >= 2 & t < 5) + deg2rad(-35)*(t >= 5 & t < 8);
        ttl    = 'Waypoints: 0° → +20° → −20° → 0°';
end

%% -- Gains -----------------------------------------------

[K_full, L, A, B, C] = compute_lqr(ref_eq, R, p, true);
K     = K_full(1:4);   % state-feedback gain  — read by Simulink
K_int = K_full(5);     % integrator gain on θ₁ error — read by Simulink

disp('Controller poles (LQI, 5-state):');
A_aug = [A, zeros(4,1); 1, 0, 0, 0, 0];
B_aug = [B; 0];
disp(sort(real(eig(A_aug - B_aug*K_full)), 'descend'))
disp('Observer poles:'); disp(sort(eig(A - L*C), 'descend'))

%% -- Reference timeseries (From Workspace payload) -------
%
%  Model works in deviation coordinates: x̂ = x − x_eq, so equilibrium = [0;0;0;0].
%  ref_x must also be in deviation coordinates: [δ; 0; −δ; 0].
%
%  Psi-coupling: θ₁_dev = +δ requires θ₂_dev = −δ so inertial arm-2 angle
%  (θ₁+θ₂) stays at π (pendulum upright) in absolute frame.

t_vec       = (0 : h : run_time)';
th1_ref_dev = arrayfun(ref_fn, t_vec);   % δ(t) — deviation from equilibrium
th2_ref_dev = -th1_ref_dev;              % −δ(t) — psi coupling

ref_x = timeseries([th1_ref_dev, zeros(size(t_vec)), th2_ref_dev, zeros(size(t_vec))], t_vec);
ref_x.Name = 'ref_x';

%% -- Run -------------------------------------------------

run_name = sprintf('ref_track_down-up_demo%d', DEMO);

sim rotpen_lqr;
save_run(0, simout, run_name);
fprintf('Saved: %s\n', run_name);

%% -- Plot ------------------------------------------------

t_out = simout.Time;
y     = simout.Data;
th1   = y(:,1);    % [deg]
th2   = y(:,3);    % [deg]
u     = y(:,6);    % [-]

% Convert deviation refs to absolute degrees for overlay on hardware output
th1_ref_deg = rad2deg(ref_eq(1)) + rad2deg(th1_ref_dev);   % 0 + δ_deg
th2_ref_deg = rad2deg(ref_eq(3)) + rad2deg(th2_ref_dev);   % 180 − δ_deg

figure(1); clf;

subplot(3,1,1);
plot(t_out, th1, 'b', t_vec, th1_ref_deg, 'k--', 'LineWidth', 1.2);
ylabel('\theta_1 [deg]'); legend('actual', 'reference'); grid on;
title(['Reference tracking — ' ttl]);

subplot(3,1,2);
plot(t_out, th2, 'r', t_vec, th2_ref_deg, 'k--', 'LineWidth', 1.2);
yline(180, 'k:');
ylabel('\theta_2 [deg]'); legend('actual', '\theta_2 ref = 180°−\theta_1'); grid on;

subplot(3,1,3);
plot(t_out, u, 'k'); yline(controller_sat, 'r--'); yline(-controller_sat, 'r--');
ylabel('u [-]'); xlabel('Time [s]'); grid on;

sgtitle(ttl);
