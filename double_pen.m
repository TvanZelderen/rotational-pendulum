function [dx, y] = double_pen(t, x, u, p, varargin)
% Double pendulum grey-box ODE for idnlgrey — with motor input
% 
% States:  x  = [th1; dth1; th2; dth2]
% Input:   u  = motor voltage [V]
% Output:  y  = [th1; dth1; th2; dth2]
%
% Parameters:
%   alpha1 = I1 + m1*l1^2 + m2*L1^2   effective inertia link 1
%   beta1  = (m1*l1 + m2*L1)*g         gravity term link 1
%   alpha2 = I2 + m2*l2^2              effective inertia link 2
%   beta2  = m2*l2*g                   gravity term link 2
%   gamma  = m2*L1*l2                  coupling inertia
%   km     = motor torque constant [Nm/V]
%   b2     = viscous damping link 2 [Nms/rad]
% {alpha1_0; beta1_0; alpha2_0; beta2_0; gamma_0; km_0; b2_0}
    alpha1 = p{1}; beta1 = p{2}; alpha2_0 = p{3}; beta2 = p{4}; gamma = p{5};  km = p{6}; b2 = p{7};

    th1 = x(1); w1 = x(2);
    th2 = x(3); w2 = x(4);
    D   = th1 - th2;

    denom = alpha1*alpha2 - gamma^2*cos(D)^2;

    % Generalised forces
    G1 = beta1*sin(th1) - km*u;   % motor torque acts on link 1
    G2 = beta2*sin(th2) + b2*w2;  % damping on free-swinging link 2

    ddth1 = (-alpha2*(gamma*w2^2*sin(D) + G1) + gamma*cos(D)*(gamma*w1^2*sin(D) - G2)) / denom;
    ddth2 = ( alpha1*(gamma*w1^2*sin(D) - G2) + gamma*cos(D)*(gamma*w2^2*sin(D) + G1)) / (-denom);

    dx = [w1; ddth1; w2; ddth2];
    y  = x;
end