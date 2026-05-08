function [A, B, C, D] = my_pendulum_model(p, T, aux)
    % p   = The 6 unknown parameters [I1, p1, p2, p3, p4, b]
    % T   = The sampling time (MATLAB sends this automatically)
    % aux = The 3 knowns [m1_val, l1_val, g_val] from your idgrey call

    % 1. Unpack parameters
    I1 = p(1); p1 = p(2); p2 = p(3); p3 = p(4); p4 = p(5); b = p(6);
    m1 = aux(1); l1 = aux(2); g = aux(3);
    % Use the formulas derived in your White Box step
    a = (m1 * g * l1) / I1; % Linearized term from Slide 10
    
    A = [0,  1,  0,  0;
         a,  0,  p1, 0; % p1 is the 'coupling' unknown
         0,  0,  0,  1;
         p2, 0,  p3, p4];
         
    B = [0; b; 0; 0]; % Direct motor effect on Link 1
    C = eye(4); D = 0;
end