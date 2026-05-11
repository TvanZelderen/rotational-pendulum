function [dx, y] = double_pen(t, x, u, p, varargin)
 % 1. Unpack Parameters (p) 
   % --- UNPACK EVERYTHING FROM P ---
    % Unknowns (MATLAB will estimate these)
    I1 = p(1); I2 = p(2); kt = p(3); c1 = p(4); c2 = p(5); 
    lc1 = p(6); lc2 = p(7);
    
    % Knowns (We will "FIX" these in the script so they don't change)
    m1 = p(8); L1 = p(9); m2 = p(10); L2 = p(11); g = p(12);

    % States
    th1 = x(1); d1 = x(2); th2 = x(3); d2 = x(4);
    diff = th1 - th2;

    % --- SCALAR DYNAMICS ---
    % We calculate the "Denominator" (the determinant of the mass matrix)
    den = (I1 + m2*L1^2) * I2 - (m2*L1*lc2*cos(diff))^2;

    % Torques (Gravity, Friction, Motor, Centrifugal)
    T1 = kt*u - c1*d1 - (m1*lc1 + m2*L1)*g*sin(th1) - m2*L1*lc2*d2^2*sin(diff);
    T2 = -c2*d2 - m2*g*lc2*sin(th2) + m2*L1*lc2*d1^2*sin(diff);

    % Accelerations (Explicitly solved)
    ddth1 = (T1 * I2 - T2 * m2 * L1 * lc2 * cos(diff)) / den;
    ddth2 = ((I1 + m2*L1^2) * T2 - m2 * L1 * lc2 * cos(diff) * T1) / den;

    dx = [d1; ddth1; d2; ddth2];
    y = x;
end