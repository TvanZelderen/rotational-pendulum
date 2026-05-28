function K = compute_lqr(x0, R, p)
% Returns LQR gain for operating point x0.
% Bryson Q: x_max = [pi, 8, pi, 26.18] rad (or rad/s)

syms x [4 1] real
syms u real

dxdt_sym = rotpen_ode(0, x, u, p);

A_sym = jacobian(dxdt_sym, x);
B_sym = jacobian(dxdt_sym, u);

A = double(subs(subs(A_sym, x, x0), u, 0));
B = double(subs(subs(B_sym, x, x0), u, 0));

Q = diag([1/pi^2, 1/8^2, 1/pi^2, 1/26.18^2]);

K = lqr(A, B, Q, R);
end
