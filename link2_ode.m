function [A, B, C, D, K] = link2_ode(param, ~)                                                                                                                                                                            
   
    alpha = param;   % c2 / (m2 * l2^2)                                                                                                                                                                                     
    beta  = 9.81/0.1;   % g / l2 = 0.1                            
  
    % These matrices are the result of a derivation with theta=0 and with the
    % small angle approximation.
    A = [0 1; -beta -alpha];                                                                                                                                                                                                                     
    B = zeros(2, 0);
    C = [1 0];
    D = zeros(1, 0);                                               
    K = zeros(2, 1);     % no noise model

end

