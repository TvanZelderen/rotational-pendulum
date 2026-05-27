clear; clc;
pendulum_params;

%% Load data first to get h
data_folder = 'data';
file_name   = '20260511_165315_doublependulum_sine_excitation.mat';
run_data    = load(fullfile(data_folder, file_name));
t_raw  = run_data.simin(:,1);
u_raw  = run_data.simin(:,2);
y_raw  = run_data.simout.Data;
h      = mean(diff(t_raw));
fprintf('h = %.4f s\n', h);

%% Linearise at stable equilibrium only
eps_jac = 1e-6;
C = [1 0 0 0; 0 0 1 0];
D = zeros(2,1);

x_eq_s = [0; 0; 0; 0];
f0     = rotpen_ode(0, x_eq_s, 0, p);
A_s    = zeros(4,4);
for i = 1:4
    xp = x_eq_s; xp(i) = xp(i) + eps_jac;
    A_s(:,i) = (rotpen_ode(0, xp, 0, p) - f0) / eps_jac;
end
B_s = zeros(4,1);
B_s = (rotpen_ode(0, x_eq_s, eps_jac, p) - f0) / eps_jac;

ev_s = eig(A_s);
fprintf('\nOpen-loop eigenvalues:\n');
for i = 1:4
    fprintf('  λ%d = %+.3f %+.3fj\n', i, real(ev_s(i)), imag(ev_s(i)));
end

%% Fresh approach — pick poles manually, well away from open-loop
% Open-loop: -3003, -0.058, -0.087±10.6j
%
% Rules:
%   1. Keep -3003 — place it exactly where it is
%   2. Move slow poles to real axis only — NO imaginary parts
%      This avoids the near-coincidence with ±10.6j
%   3. Keep |pole| * h < 0.3 for all poles

fprintf('\nOpen-loop poles:\n'); disp(ev_s)

% All real poles — no imaginary parts at all
% Slightly faster than slowest open-loop pole (-0.058)
obs_poles = [-2; -4; -5; -3005];

fprintf('Candidate observer poles:\n'); disp(obs_poles)
fprintf('Pole*h check:\n');
for i = 1:4
    fprintf('  %.1f * %.3f = %.4f\n', obs_poles(i), h, abs(obs_poles(i))*h);
end

%% Try place with these poles
L_s = place(A_s', C', obs_poles)';
L_s = real(L_s);
fprintf('Max |L| = %.4f\n', max(abs(L_s(:))));

%% Verify stability
ev_obs = eig(A_s - L_s*C);
fprintf('Observer eigenvalues:\n');
for i = 1:4
    fprintf('  %+.4f %+.4fj\n', real(ev_obs(i)), imag(ev_obs(i)));
end
if all(real(ev_obs) < 0)
    fprintf('Observer STABLE\n');
    fprintf('\nPaste into observer_step.m:\n');
    fprintf('L = [%.6f  %.6f\n', L_s(1,1), L_s(1,2));
    fprintf('     %.6f  %.6f\n', L_s(2,1), L_s(2,2));
    fprintf('     %.6f  %.6f\n', L_s(3,1), L_s(3,2));
    fprintf('     %.6f  %.6f];\n', L_s(4,1), L_s(4,2));
else
    fprintf('UNSTABLE — try slower poles\n');
    % If still unstable, try:
    % obs_poles = [-0.2; -0.3; -0.4; -3003];
end


%L_s =  [ 13.2692 2.4922;
%         -104.1660 -83.3741;
%         -15.9139 0.9017;
%          17.9635 165.8050];
%% Process data
th1_rad  = deg2rad(y_raw(:,1));
dth1_rad = deg2rad(y_raw(:,2));
th2_rad  = deg2rad(y_raw(:,3));
dth2_rad = deg2rad(y_raw(:,4));

iS = round(3/h)+1;
iE = min(round(28/h)+1, length(t_raw));

t_trim    = t_raw(iS:iE);
u_trim    = u_raw(iS:iE);
th1_trim  = th1_rad(iS:iE);
dth1_trim = dth1_rad(iS:iE);
th2_trim  = th2_rad(iS:iE);
dth2_trim = dth2_rad(iS:iE);
N         = length(t_trim);

%% Run observer
x_hat      = zeros(4, N);
x_hat(:,1) = [th1_trim(1); 0; th2_trim(1); 0];

for k = 1:N-1
    if any(isnan(x_hat(:,k))) || any(abs(x_hat(:,k)) > 1e4)
        fprintf('Diverged at t=%.2f s\n', t_trim(k));
        x_hat(:,k+1:end) = NaN;
        break
    end

    y_meas = [th1_trim(k); th2_trim(k)];
    innov  = y_meas - C * x_hat(:,k);
    xk     = x_hat(:,k);
    uk     = u_trim(k);

    % RK4
    %k1 = rotpen_ode(0, xk,         uk, p) + L_s*innov;
    %k2 = rotpen_ode(0, xk+h/2*k1, uk, p) + L_s*innov;
    %k3 = rotpen_ode(0, xk+h/2*k2, uk, p) + L_s*innov;
    %k4 = rotpen_ode(0, xk+h*k3,   uk, p) + L_s*innov;

    %x_hat(:,k+1) = xk + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
    
    k1 = rotpen_ode(0, xk,          uk, p) + L_s * (y_meas - C * xk);
    k2 = rotpen_ode(0, xk + h/2*k1, uk, p) + L_s * (y_meas - C * (xk + h/2*k1));
    k3 = rotpen_ode(0, xk + h/2*k2, uk, p) + L_s * (y_meas - C * (xk + h/2*k2));
    k4 = rotpen_ode(0, xk + h*k3,   uk, p) + L_s * (y_meas - C * (xk + h*k3));

    x_hat(:,k+1) = xk + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
end

%% Plot
figure;
labels = {'th1 [deg]','dth1 [deg/s]','th2 [deg]','dth2 [deg/s]'};
true_s = {rad2deg(th1_trim), rad2deg(dth1_trim), ...
          rad2deg(th2_trim), rad2deg(dth2_trim)};

for i = 1:4
    subplot(4,1,i);
    plot(t_trim, true_s{i}, 'b', ...
         t_trim, rad2deg(x_hat(i,:))', 'r--', 'LineWidth', 1.2);
    ylabel(labels{i}, 'Interpreter','none'); grid on;
    if i==1
        legend('Measured','Observer','Location','best');
        title('Nonlinear Luenberger Observer — Stable eq');
    end
end
xlabel('Time [s]');

%% Convergence
vel_err_1 = rms(rad2deg(dth1_trim) - rad2deg(x_hat(2,:))');
vel_err_2 = rms(rad2deg(dth2_trim) - rad2deg(x_hat(4,:))');
fprintf('\nRMS velocity error: dth1=%.2f deg/s  dth2=%.2f deg/s\n', ...
        vel_err_1, vel_err_2);

for test_poles = {[-2.0+15j; -2.0-15j; -2.0; -251], ...
                  [-2.0; -2.5; -3.0; -251], ...
                  [-1.5+11j; -12.5; -1.5; -251]}
    L_test = place(A_s', C', test_poles{1})';
    fprintf('Max |L| = %.2f\n', max(abs(L_test(:))));
end

%% Check 1 — what does the ODE output at the initial state?
x0_test = [th1_trim(1); 0; th2_trim(1); 0];
u0_test  = u_trim(1);
dx_test  = rotpen_ode(0, x0_test, u0_test, p);
fprintf('Initial state:\n');
fprintf('  th1  = %.4f rad (%.1f deg)\n', x0_test(1), rad2deg(x0_test(1)));
fprintf('  dth1 = %.4f rad/s\n', x0_test(2));
fprintf('  th2  = %.4f rad (%.1f deg)\n', x0_test(3), rad2deg(x0_test(3)));
fprintf('  dth2 = %.4f rad/s\n', x0_test(4));
fprintf('ODE output dx:\n');
fprintf('  ddth1 = %.4f rad/s2\n', dx_test(2));
fprintf('  ddth2 = %.4f rad/s2\n', dx_test(4));

%% Check 2 — simulate ODE alone for 1 second (no observer)
odefun = @(t,x) rotpen_ode(t, x, ...
    interp1(t_trim, u_trim, t, 'linear', 'extrap'), p);
odeOpts = odeset('RelTol',1e-6,'AbsTol',1e-8);

[t_ode, x_ode] = ode45(odefun, t_trim(1:min(100,length(t_trim))), x0_test, odeOpts);
figure;
subplot(2,1,1);
plot(t_ode, rad2deg(x_ode(:,1)), t_ode, rad2deg(x_ode(:,3)));
legend('th1','th2'); ylabel('deg'); title('ODE only — no observer');
subplot(2,1,2);
plot(t_ode, x_ode(:,2), t_ode, x_ode(:,4));
legend('dth1','dth2'); ylabel('rad/s'); xlabel('Time [s]');

%% Print all relevant parameters
fprintf('\n--- pendulum_params check ---\n');
fprintf('m1   = %.4f kg\n',    p.m1);
fprintf('m2   = %.4f kg\n',    p.m2);
fprintf('l1   = %.4f m\n',     p.l1);
fprintf('l2   = %.4f m\n',     p.l2);
fprintf('lc1  = %.4f m\n',     p.lc1);
fprintf('J1   = %.6f kg.m2\n', p.J1);
fprintf('g    = %.4f m/s2\n',  p.g);
fprintf('km   = %.4f Nm\n',    p.km);
fprintf('kbc1 = %.4f Nms\n',   p.kbc1);
fprintf('c2   = %.6f Nms\n',   p.c2);

%% Check mass matrix at equilibrium
th2_eq = 0;
M_eq = [p.J1 + p.m2*(p.l1^2 + p.l2^2 + 2*p.l1*p.l2*cos(th2_eq)), ...
        p.m2*(p.l2^2 + p.l1*p.l2*cos(th2_eq)); ...
        p.m2*(p.l2^2 + p.l1*p.l2*cos(th2_eq)), ...
        p.m2*p.l2^2];

fprintf('\nMass matrix at equilibrium:\n'); disp(M_eq)
fprintf('det(M) = %.6e\n', det(M_eq));
fprintf('cond(M) = %.2e\n', cond(M_eq));

%% Expected acceleration from gravity alone at th1=10deg, th2=-7deg
g_torque1 = (p.m1*p.lc1 + p.m2*p.l1)*p.g*sin(deg2rad(10));
g_torque2 = p.m2*p.l2*p.g*sin(deg2rad(-7));
fprintf('\nGravity torque on link1: %.4f Nm\n', g_torque1);
fprintf('Gravity torque on link2: %.4f Nm\n', g_torque2);
fprintf('Expected ddth1 ~ %.2f rad/s2 (should be < 50)\n', g_torque1/M_eq(1,1));
fprintf('Expected ddth2 ~ %.2f rad/s2 (should be < 50)\n', g_torque2/M_eq(2,2));

%%
fprintf('u range in data: %.4f to %.4f\n', min(u_trim), max(u_trim));
fprintf('Motor torque range: %.4f to %.4f Nm\n', p.km*min(u_trim), p.km*max(u_trim));
fprintf('Expected ddth1 from motor: %.1f to %.1f rad/s2\n', ...
        p.km*min(u_trim)/M_eq(1,1), p.km*max(u_trim)/M_eq(1,1));

%% Inspect the data file properly
fprintf('u unique values: %d\n', length(unique(u_raw)));
fprintf('u min=%.4f  max=%.4f  mean=%.4f\n', min(u_raw), max(u_raw), mean(u_raw));
fprintf('t range: %.2f to %.2f s\n', t_raw(1), t_raw(end));
fprintf('th1 range: %.1f to %.1f deg\n', min(y_raw(:,1)), max(y_raw(:,1)));
fprintf('th2 range: %.1f to %.1f deg\n', min(y_raw(:,3)), max(y_raw(:,3)));

%% Plot raw data to see what it actually is
figure;
subplot(3,1,1); plot(t_raw, y_raw(:,1)); ylabel('th1 [deg]'); grid on;
subplot(3,1,2); plot(t_raw, y_raw(:,3)); ylabel('th2 [deg]'); grid on;
subplot(3,1,3); plot(t_raw, u_raw);      ylabel('u [-]');     grid on;
xlabel('Time [s]');
title('Raw data — checking what this file actually contains');