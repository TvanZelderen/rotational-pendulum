function [dx, y] = double_pen2(t, x, u, m1, l1, I1, km, b1, varargin)
% Uses mass matrix form: M*ddth = f  solved via M\f
% This avoids explicit denom which goes singular

    % Fixed from single pendulum
    alpha2 = 9.87e-4;
    beta2  = 0.0975;
    gamma0 = 9.94e-4;   % rename to avoid conflict
    b2     = 3.21e-4;
    g      = 9.81;
    m2     = 0.10;
    L1     = 0.10;

    th1 = x(1); w1 = x(2);
    th2 = x(3); w2 = x(4);
    D   = th1 - th2;

    % alpha1 from estimated physical params
    alpha1 = I1 + m1*l1^2 + m2*L1^2;
    beta1  = (m1*l1 + m2*L1)*g;

    % Mass matrix M (2x2, always positive definite if params physical)
    M = [alpha1,              gamma0*cos(D); ...
         gamma0*cos(D),       alpha2       ];

    % Right-hand side forces
    f1 = -gamma0*w2^2*sin(D) - beta1*sin(th1) + km*u - b1*w1;
    f2 =  gamma0*w1^2*sin(D) - beta2*sin(th2)         - b2*w2;

    % Solve M * [ddth1; ddth2] = [f1; f2]
    ddth = M \ [f1; f2];

    dx = [w1; ddth(1); w2; ddth(2)];
    y  = x;
end