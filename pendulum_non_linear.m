function [dx, y] = pendulum_non_linear(t, x, u, I1, p1, p2, p3, p4, b, varargin)
    % x = [th1, dth1, th2, dth2]
    % u = motor current (Amperes)
    % p1-p4 = coupling/black-box parameters
    
    % Access your known measurements (m1, l1, g) from varargin
    aux = varargin{1};
    m1 = aux(1); l1 = aux(2); g = aux(3);

    % 1. Define the Non-linear Equations of Motion
    % Use sin(x(1)) instead of the linear 'a * x1'
    d_th1  = x(2);
    d_dth1 = -(m1 * g * l1 / I1) * sin(x(1)) + p1 * x(3) + b * u;
    
    d_th2  = x(4);
    % You can make the second link non-linear too if you want:
    d_dth2 = p2 * sin(x(1)) + p3 * sin(x(3)) + p4 * x(4);

    % 2. State derivatives vector
    dx = [d_th1; d_dth1; d_th2; d_dth2];

    % 3. Output equation (what the sensors see)
    y = [x(1); x(2); x(3); x(4)]; 
end