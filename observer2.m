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

%% Check observability
fprintf('\nObservability rank: %d/4\n', rank(obsv(A_s, C)));

%% The fix — use acker() instead of place()
% place() struggles with widely spread poles like -0.15 and -251
% acker() handles this better for SISO-like cases
% BUT: with 2 outputs we need a different approach

% Strategy: ignore the -251 pole (motor pole, already fast)
% Design observer only for the 3 slow poles, fix the fast one
fprintf('\nNote: pole at -251 is already very fast (motor)\n');
fprintf('Designing observer to handle slow poles only\n');

% Use these poles — slightly faster than open-loop slow poles
% Open-loop slow poles: -0.698, -0.146±10.6j
% Observer poles: 5x faster
obs_poles = [-0.1+11j; -0.1; -0.1-11j; -3005];
% Keep the -251 pole where it is — dont move it!
% Only move the 3 slow poles

fprintf('\nObserver poles:\n'); disp(obs_poles)

L_s = place(A_s', C', obs_poles)'
fprintf('Max |L| = %.2f\n', max(abs(L_s(:))));

ev_obs = eig(A_s - L_s*C);
fprintf('Observer eigenvalues:\n'); disp(ev_obs)
if all(real(ev_obs) < 0)
    fprintf('Observer STABLE\n');
else
    error('Observer unstable!');
end

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