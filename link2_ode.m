function [A, B, C, D, K] = link2_ode(params, ~)                                                                                                                                                                            
   
    alpha = params(1);   % c2 / (m2 * l2^2)                                                                                                                                                                                     
    beta  = params(2);   % g / l2                             
  
    % These matrices are the result of a derivation with theta=0 and with the
    % small angle approximation.
    A = [0 1; -beta -alpha];                                                                                                                                                                                                                     
    B = [0; 0];
    C = [1 0];
    D = 0;                                                   
    K = zeros(2, 1);     % no noise model

    end