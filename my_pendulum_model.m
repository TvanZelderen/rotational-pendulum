function [dx, y] = my_pendulum_model(t, x, u, p, varargin)
    % p = [I1; I2; kt; c1; c2] (The 5 Unknowns)
    I1 = p(1); I2 = p(2); kt = p(3); c1 = p(4); c2 = p(5);
    lc1 = p(6); lc2 = p(7);
    
    % aux = [m1, L1, lc1, m2, L2, lc2, g] (The 7 Measured Knowns)
    aux = varargin{1};
    m1 = aux(1); L1 = aux(2); 
    m2 = aux(3); L2 = aux(4); g = aux(5);

    % States: x = [th1; dth1; th2; dth2]
    th1 = x(1); dth1 = x(2); th2 = x(3); dth2 = x(4);

    % 1. Mass Matrix M(q)
    M11 = I1 + m2 * L1^2;
    M12 = m2 * L1 * lc2 * cos(th1 - th2);
    M22 = I2;
    M = [M11, M12; M12, M22];

    % 2. Force Vector F(q, dq, u)
    F1 = kt*u - c1*dth1 - (m1*lc1 + m2*L1)*g*sin(th1) - m2*L1*lc2*dth2^2*sin(th1 - th2);
    F2 = -c2*dth2 - m2*g*lc2*sin(th2) + m2*L1*lc2*dth1^2*sin(th1 - th2);
    F = [F1; F2];

    % 3. Solve for accelerations
    accel = M \ F; % Returns [ddth1; ddth2]

    dx = [dth1; accel(1); dth2; accel(2)];
    y  = x; % We output all 4 states for the data fit
end