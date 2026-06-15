function [K,L,A,B,C] = compute_lqr(x0, R, p)
% Returns LQR gain for operating point x0.
% Bryson Q: x_max = [pi, 8, pi, 26.18] rad (or rad/s)

syms x [4 1] real
syms u real

dxdt_sym = rotpen_ode(0, x, u, p);

A_sym = jacobian(dxdt_sym, x);
B_sym = jacobian(dxdt_sym, u);

A = double(subs(subs(A_sym, x, x0), u, 0));
B = double(subs(subs(B_sym, x, x0), u, 0));
C = [1 0 0 0; 
     0 0 1 0]; 
D = zeros(2,1);
% Bryson baseline
Q_base = diag([100, 0, 0, 0]);
% Q_base = diag([1/pi^2, 1/8^2, 1/pi^2, 1/26^2]);

% Penalise psi = th1 + th2 (arm-2 inertial angle deviation)
c_psi = [1 0 1 0];
w_psi = 200;   % TUNABLE — raise to stiffen arm-2-up, watch u vs ±sat
Q = Q_base + w_psi * (c_psi' * c_psi)
% Q = Q_base;

K = lqr(A, B, Q, R);

%observer

% Build Q and R
Q_obs = diag([800, 5000 ,800 ,5000]);
% Q_obs = diag([1000,5000,1000,5000]);

R_obs = 0.01*diag([1,1]);

% N — cross-covariance between process and measurement noise
% Usually zero
N = zeros(4,2);

%% Build state space model
sys = ss(A, [B, eye(4)], C, [D, zeros(2,4)]);
%          ↑              ↑
%     control input   process noise input (identity = noise on all states)

%% Compute Kalman gain
[~, L, ~] = kalman(sys, Q_obs, R_obs);

end


