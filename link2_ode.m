function [dx, y] = link2_ode(t, x, u, alpha, beta, varargin)
% Nonlinear grey-box ODE for link 2 (free pendulum), for use with idnlgrey.
%   alpha = c2 / (m2 * l2^2)  [rad/s — damping]
%   beta  = g / l2             [rad/s^2 — gravity]
%   x(1)  = theta2             [rad]
%   x(2)  = dtheta2/dt         [rad/s]

    dx = [x(2);
          -beta * sin(x(1)) - alpha * x(2)];
    y  = x(1);

end
