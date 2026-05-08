function [dx, y] = pendulum_non_linear(t, x, u, p, varargin)
    % t = time, x = states, u = input
    % p = The parameter vector [I1; p1; p2; p3; p4; b]
    
    % 1. Unpack the Unknowns from the vector 'p'
    I1 = p(1); p1 = p(2); p2 = p(3); p3 = p(4); p4 = p(5); b = p(6);

    % 2. Unpack the Knowns from the varargin suitcase
    aux = varargin{1}; 
    m1 = aux(1); l1 = aux(2); g = aux(3);

    % 3. Non-linear Dynamics (The Curvy Physics)
    d_th1  = x(2);
    % Main Equation: ddth1 = - (mgl/I) * sin(th1) + p1 * sin(th2) + b * u
    d_dth1 = -(m1 * g * l1 / I1) * sin(x(1)) + p1 * sin(x(3)) + b * u;
    
    d_th2  = x(4);
    % Coupling Equation: ddth2 = p2 * sin(th1) + p3 * sin(th2) + p4 * dth2
    d_dth2 = p2 * sin(x(1)) + p3 * sin(x(3)) + p4 * x(4);

    % 4. Assemble output
    dx = [d_th1; d_dth1; d_th2; d_dth2];
    y = x; % Sensors measure all 4 states directly
end