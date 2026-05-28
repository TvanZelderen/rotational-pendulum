%% —— Setup —————————————————————————————————————
% Create the symbolic vectors
syms x [4 1] real
syms u real

% Get the system parameters
pendulum_params;

% Load rotpen ode
dxdt_sym = rotpen_ode(0, x, u, p);

% Equilibrium position
% x0 = [0; 0; pi; 0]; % down up
x0 = [pi; 0; 0; 0]; % up up
u0 = 0;
ref = x0;

%% —— Jacobian ——————————————————————————————————
% ↓  jacobian(dxdt_sym, x_sym) → A_sym
% ↓  jacobian(dxdt_sym, u_sym) → B_sym
% ↓  subs + double() → numeric A, B at x0

A_sym = jacobian(dxdt_sym, x);
B_sym = jacobian(dxdt_sym, u);

A_num = double(subs(subs(A_sym, x, x0), u, u0));
B_num = double(subs(subs(B_sym, x, x0), u, u0));

C = [1, 0, 0, 0; 0, 0, 1, 0];

% % Sanity checks
% eig(A_num)
% rank(ctrb(A_num, B_num))
% rank(obsv(A_num, C))

%% —— LQR ———————————————————————————————————————

Q = [1, 0, 0, 0; 0, 0.5, 0, 0; 0, 0, 2, 0; 0, 0, 0, 1]; % just a first guess
R = 0.1; % aggressive starting point

% Compute the LQR gain matrix
K = lqr(A_num, B_num, Q, R);

% % Sanity check
% eig(A_num - B_num*K)