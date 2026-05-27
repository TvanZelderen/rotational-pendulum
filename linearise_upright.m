%% —— Setup —————————————————————————————————————
% Create the symbolic vectors
syms x [4 1] real
syms u real

% Get the system parameters
pendulum_params;

% Load rotpen ode
dxdt_sym = rotpen_ode(0, x, u, p);

%% —— Jacobian ——————————————————————————————————
% ↓  jacobian(dxdt_sym, x_sym) → A_sym
% ↓  jacobian(dxdt_sym, u_sym) → B_sym
% ↓  subs + double() → numeric A, B at x0

