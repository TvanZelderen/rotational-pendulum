clear; clc;
calib;
hwinit;
pendulum_params;

h = 0.001;
R = 5;
ref = [pi; 0; 0; 0];
[K, L, A, B, C] = compute_lqr(ref, R, p);


run_time = 60;

sim rotpentemplate;