%% Pure MATLAB MPC closed-loop test
pendulum_params;
Ts  = 0.001;
ref = [0; 0; 0; 0];
x0  = ref;   % your operating point
R = 5;
% Recompute MPC
[mpc_obj, Ad, Bd, Cd] = compute_mpc(ref, R, p);

% Initial condition — 20 deg perturbation on th2
x     = x0 + [0; 0; deg2rad(20); 0];
C_mpc = [1 0 0 0; 0 0 1 0];

Tsim  = 3;
t_sim = 0:Ts:Tsim;
n_sim = length(t_sim);
xlog  = zeros(4, n_sim);
ulog  = zeros(1, n_sim);

mpc_state = mpcstate(mpc_obj);

for k = 1:n_sim
    xlog(:,k) = x;

    % Deviation from nominal
    y_dev = C_mpc*x - C_mpc*x0;
    r_dev = zeros(2,1);

    % MPC step
    [u, info] = mpcmove(mpc_obj, mpc_state, y_dev, r_dev);
    ulog(k)   = u;

    fprintf('k=%d  th2=%.2f deg  u=%.4f  qp=%d\n', ...
            k, rad2deg(x(3)), u, info.QPCode);

    % Nonlinear ODE step
    k1 = rotpen_ode(0, x,          u, p);
    k2 = rotpen_ode(0, x+Ts/2*k1,  u, p);
    k3 = rotpen_ode(0, x+Ts/2*k2,  u, p);
    k4 = rotpen_ode(0, x+Ts*k3,    u, p);
    x  = x + (Ts/6)*(k1+2*k2+2*k3+k4);

    if any(isnan(x)) || any(abs(x) > 100)
        fprintf('Simulation diverged at k=%d\n', k); break
    end
end

figure;
subplot(3,1,1);
plot(t_sim(1:k), rad2deg(xlog(1,1:k)), ...
     t_sim(1:k), rad2deg(xlog(3,1:k)));
legend('th1','th2'); ylabel('deg'); grid on;
title('MPC pure MATLAB test');
subplot(3,1,2);
plot(t_sim(1:k), rad2deg(xlog(2,1:k)), ...
     t_sim(1:k), rad2deg(xlog(4,1:k)));
legend('dth1','dth2'); ylabel('deg/s'); grid on;
subplot(3,1,3);
plot(t_sim(1:k), ulog(1:k));
ylabel('u [-]'); xlabel('Time [s]'); grid on;
yline(mpc_obj.MV.Max,'r--'); yline(mpc_obj.MV.Min,'r--');