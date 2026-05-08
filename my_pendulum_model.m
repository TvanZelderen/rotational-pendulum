function [A, B, C, D] = my_pendulum_model(I1, p1, p2, p3, p4, b, T, aux)
    % aux contains [m1, l1, g] which you measured in the lab
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