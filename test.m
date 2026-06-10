%% Stop all parallel output — add this at top of test script
diary off;
warning off;

%% Clean MPC test — minimal and explicit
clear mpc_obj mpc_state;
pendulum_params;

Ts = 0.001;
x0 = [0;0;0;0];   % test at stable equilibrium first

%% Manually linearise
eps_j = 1e-6;
A = zeros(4,4);
f0 = rotpen_ode(0, x0, 0, p);
for i = 1:4
    xp = x0; xp(i) = xp(i)+eps_j;
    xm = x0; xm(i) = xm(i)-eps_j;
    A(:,i) = (rotpen_ode(0,xp,0,p) - rotpen_ode(0,xm,0,p))/(2*eps_j);
end
B = (rotpen_ode(0,x0,eps_j,p) - rotpen_ode(0,x0,-eps_j,p))/(2*eps_j)

%% Only 2 outputs — angles only
C_mpc = [1 0 0 0;
         0 0 1 0];
D_mpc = zeros(2,1);

%% Check eigenvalues
fprintf('Open-loop poles:\n'); disp(eig(A))

%% Discretise
sys_c = ss(A, B, C_mpc, D_mpc);
sys_d = c2d(sys_c, Ts, 'zoh');

%% Verify discrete system is stable/correct
fprintf('Discrete eigenvalues (should be inside unit circle for stable):\n');
disp(eig(sys_d.A))

%% Build MPC — simple settings first
Np = 200;
Nc = 20;
mpc_obj = mpc(sys_d, Ts, Np, Nc);

% Check scale factors first
mpc_obj.OV(1).ScaleFactor = pi;   % th1 in radians, range ~ pi
mpc_obj.OV(2).ScaleFactor = pi;   % th2 in radians, range ~ pi
mpc_obj.MV.ScaleFactor    = 1;    % u normalised [-1,1]

% Constraints
mpc_obj.MV.Min = -1;
mpc_obj.MV.Max =  1;

% Weights
mpc_obj.Weights.OutputVariables          = [1, 1];
mpc_obj.Weights.ManipulatedVariables     = 0.1;
mpc_obj.Weights.ManipulatedVariablesRate = 0.001;

% Nominal point
mpc_obj.Model.Nominal.X = x0;
mpc_obj.Model.Nominal.Y = C_mpc * x0;
mpc_obj.Model.Nominal.U = 0;

%% Single step test — check u is in [-1,1]
mpc_state = mpcstate(mpc_obj);
x_test    = x0 + [0;0;deg2rad(10);0];
y_dev     = C_mpc*x_test - C_mpc*x0;
r_dev     = zeros(2,1);
[u_test, info] = mpcmove(mpc_obj, mpc_state, y_dev, r_dev);

fprintf('\n=== Single step test ===\n');
fprintf('th2 perturbation: 10 deg\n');
fprintf('u output:  %.6f  (should be in [-1,1])\n', u_test);
fprintf('QP status: %d    (1=optimal, 0=infeasible)\n', info.QPCode);

if abs(u_test) > 1
    fprintf('ERROR: u out of range — scale factor or unit problem\n');
    fprintf('Check: is B in correct units?\n');
    fprintf('B = %.6f  (torque per unit input)\n', B(2));
elseif abs(u_test) < 0.001
    fprintf('WARNING: u very small — output weights may be too low\n');
else
    fprintf('OK: u in reasonable range\n');
end

%% Short simulation — 1 second only
fprintf('\n=== 1 second simulation ===\n');
x         = x0 + [0;0;deg2rad(10);0];
mpc_state = mpcstate(mpc_obj);
t_end     = 1.0;
n_steps   = round(t_end/Ts);
xlog      = zeros(4, n_steps);
ulog      = zeros(1, n_steps);

for k = 1:n_steps
    xlog(:,k) = x;
    y_dev     = C_mpc*x - C_mpc*x0;
    [u, ~]    = mpcmove(mpc_obj, mpc_state, y_dev, zeros(2,1));
    u         = max(-1, min(1, u));
    ulog(k)   = u;

    k1 = rotpen_ode(0, x,          u, p);
    k2 = rotpen_ode(0, x+Ts/2*k1,  u, p);
    k3 = rotpen_ode(0, x+Ts/2*k2,  u, p);
    k4 = rotpen_ode(0, x+Ts*k3,    u, p);
    x  = x + (Ts/6)*(k1+2*k2+2*k3+k4);
end

t_plot = (0:n_steps-1)*Ts;
figure;
subplot(2,1,1);
plot(t_plot, rad2deg(xlog(3,:)));
ylabel('th2 [deg]'); grid on;
title('MPC test — th2 should return to 0');
yline(0,'r--');
subplot(2,1,2);
plot(t_plot, ulog);
ylabel('u [-]'); xlabel('Time [s]'); grid on;
yline(1,'r--'); yline(-1,'r--');

fprintf('Final th2: %.2f deg (started at 10 deg)\n', rad2deg(xlog(3,end)));
if abs(rad2deg(xlog(3,end))) < abs(rad2deg(xlog(3,1)))
    fprintf('th2 is converging\n');
else
    fprintf('th2 is diverging — model or weight problem\n');
end