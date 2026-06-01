pendulum_params;
R = 50; % Increase if unstable on hardware

[K_up_up,L]   = compute_lqr([pi; 0; 0; 0], R, p);
[K_down_up,~] = compute_lqr([0; 0; pi; 0], R, p);


save('lqr_gains.mat', 'K_up_up', 'K_down_up','L');
