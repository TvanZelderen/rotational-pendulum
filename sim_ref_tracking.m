% sim_ref_tracking.m  —  LQR+I reference tracking demos
%
% Arm 1 tracks a time-varying setpoint while the pendulum stays upright.
%
% DEMO  1  step       — arm jumps +45° at t = 3 s
% DEMO  2  sine       — arm sweeps ±30° at 0.25 Hz
% DEMO  3  waypoints  — arm visits +60°, then −60°, then returns to 0°

clear; clc;
pendulum_params;

%% -----------------------------------------------------------------------
%  Config
% -----------------------------------------------------------------------
DEMO   = 3;               % 1 | 2 | 3

ref_eq = [0; 0; pi; 0];   % down-up equilibrium (arm 1 at 0, pendulum up)
R      = 25;
h      = 0.001;           % log / resample step [s]

[K_full, ~, ~, ~, ~] = compute_lqr(ref_eq, R, p, true);
K_fb  = K_full(1:4);      % state-feedback gain
K_int = K_full(5);        % integrator gain on θ₁ error

%% -----------------------------------------------------------------------
%  Reference trajectory  (offset from ref_eq(1) = 0)
% -----------------------------------------------------------------------
switch DEMO
    case 1
        Tsim   = 12;
        ref_fn = @(t) deg2rad(30) * (t >= 3);
        ttl    = 'Step: arm 1 +30° at t = 3 s';
    case 2
        Tsim   = 12;
        ref_fn = @(t) deg2rad(30) * sin(2*pi * 0.25 * t);
        ttl    = 'Sine sweep: ±30°, 0.25 Hz';
    case 3
        Tsim   = 13;
        ref_fn = @(t) deg2rad(20)*(t >= 2 & t < 5) + deg2rad(-20)*(t >= 5 & t < 8);
        ttl    = 'Waypoints: 0° → +20° → −20° → 0°';
end

%% -----------------------------------------------------------------------
%  Augmented ODE:  xa = [θ₁, dθ₁, θ₂, dθ₂, z_int]
%  z_int = ∫(θ₁ − θ₁_ref) dt   (LQI integrator state)
% -----------------------------------------------------------------------
sat    = @(u) max(-1, min(1, u));
ref_x  = @(t) [ref_eq(1) + ref_fn(t); ref_eq(2); ref_eq(3) - ref_fn(t); ref_eq(4)];
u_fn   = @(t, xa) sat(-K_fb * (xa(1:4) - ref_x(t)) - K_int * xa(5));
odefun = @(t, xa) [rotpen_ode(t, xa(1:4), u_fn(t, xa), p); ...
                   xa(1) - (ref_eq(1) + ref_fn(t))];

x0_aug = [ref_eq + [deg2rad(2); 0; deg2rad(3); 0]; 0];

opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', h);
[t_out, x_out] = ode45(odefun, [0, Tsim], x0_aug, opts);

%% -----------------------------------------------------------------------
%  Resample onto uniform grid and reconstruct u
% -----------------------------------------------------------------------
t        = (0 : h : Tsim)';
xq       = interp1(t_out, x_out, t);
th1_ref  = ref_eq(1) + arrayfun(ref_fn, t);
u_log    = arrayfun(@(k) u_fn(t(k), xq(k,:)'), (1:length(t))');

%% -----------------------------------------------------------------------
%  Plot
% -----------------------------------------------------------------------
figure(1); clf;

subplot(3,1,1);
plot(t, rad2deg(xq(:,1)), 'b', t, rad2deg(th1_ref), 'k--', 'LineWidth', 1.2);
ylabel('\theta_1 [deg]');
legend('actual', 'reference', 'Location', 'best');
grid on;
title(['Reference tracking — ' ttl]);

subplot(3,1,2);
plot(t, rad2deg(xq(:,3)), 'r');
yline(rad2deg(ref_eq(3)), 'k--');
ylabel('\theta_2 [deg]');
legend('\theta_2', 'ref (up)', 'Location', 'best');
grid on;

subplot(3,1,3);
plot(t, u_log, 'k');
yline( 1, 'r--');
yline(-1, 'r--');
ylabel('u [-]');
xlabel('Time [s]');
grid on;

sgtitle(ttl);

%% -----------------------------------------------------------------------
%  Pack simout for animate_pendulum
%  Columns: [th1_deg, dth1_dps, th2_deg, dth2_dps, psi_deg]
% -----------------------------------------------------------------------
th1_deg  = rad2deg(xq(:,1));
dth1_dps = rad2deg(xq(:,2));
th2_deg  = rad2deg(xq(:,3));
dth2_dps = rad2deg(xq(:,4));
psi_deg  = th1_deg + th2_deg;

simout        = timeseries([th1_deg, dth1_dps, th2_deg, dth2_dps, psi_deg], t);
simout.Name   = 'simout';

animate_pendulum(simout, p);
