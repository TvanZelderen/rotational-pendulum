function [K,L,A,B,C] = compute_lqr(x0, R, p, use_lqi)
% Returns LQR gain for operating point x0.
% use_lqi = true  → K is 1×5, last element is integral gain on θ₁
% use_lqi = false → K is 1×4 (standard LQR)

if nargin < 4, use_lqi = false; end

syms x [4 1] real
syms u real

dxdt_sym = rotpen_ode(0, x, u, p);

A_sym = jacobian(dxdt_sym, x);
B_sym = jacobian(dxdt_sym, u);

A = double(subs(subs(A_sym, x, x0), u, 0))
B = double(subs(subs(B_sym, x, x0), u, 0))
C = [1 0 0 0;
     0 0 1 0];
D = zeros(2,1);

% Bryson baseline
Q_base = diag([100, 0, 0, 0]);

% Penalise psi = th1 + th2 (arm-2 inertial angle deviation)
c_psi = [1 0 1 0];
w_psi = 200;   % TUNABLE — raise to stiffen arm-2-up, watch u vs ±sat
Q = Q_base + w_psi * (c_psi' * c_psi)

if use_lqi
    % Augment state with z = ∫(θ₁ - θ₁_ref) dt
    % ż = θ₁ - θ₁_ref  →  augmented error dynamics: ż = C_int*(x-x_ref)
    C_int = [1, 0, 0, 0];
    A_aug = [A,      zeros(4,1);
             C_int,  0         ];
    B_aug = [B; 0];
    q_int = 20;   % TUNABLE — integral weight (raise for faster centering, lower for less overshoot)
    Q_aug = blkdiag(Q, q_int);
    K = lqr(A_aug, B_aug, Q_aug, R);   % 1×5: [K_x(1:4), K_int]
else
    K = lqr(A, B, Q, R);               % 1×4
end

% Observer (4-state, unchanged regardless of LQI)
Q_obs = diag([800, 5000, 800, 5000]);
R_obs = 0.01*diag([1,1]);

sys = ss(A, [B, eye(4)], C, [D, zeros(2,4)]);
[~, L, ~] = kalman(sys, Q_obs, R_obs);

end


