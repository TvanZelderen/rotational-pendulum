%% -- Preamble --------------------------------------------

clear; clc;
calib;
hwinit;
pendulum_params;

h = 0.001;
R = 50;
controller_sat = 0.5;
ref = [pi; 0; 0; 0];
[K, L, A, B, C] = compute_lqr(ref, R, p);

run_time = 20;

%% -- Run -------------------------------------------------
sim rotpentemplate;
save_run(0, simout, sprintf('lqr03dmHz'));

%% -- Plot ------------------------------------------------
t_out = simout.Time;
y = simout.Data;
th1 = y(:,1);
dth1 = y(:,2);
th2 = y(:,3);
dth2 = y(:,4);
phi = y(:,5);
oth1 = y(:,6);
odth1 = y(:,7);
oth2 = y(:,8);
odth2 = y(:,9);
input  = y(:,10);
error = y(:,11);

figure(4); clf;
    subplot(4,1,1); plot(t_out, th1, t_out, oth1); legend('\theta_1', '\theta_1_ob'); ylabel('\theta_1 [deg]'); grid on;
    subplot(4,1,2); plot(t_out, dth1, t_out, odth1); legend('\theta_1_d', '\theta_1_dob'); ylabel('d\theta_1'); grid on;
    subplot(4,1,3); plot(t_out, th2, t_out, oth2); legend('\theta_2', '\theta_2_ob'); ylabel('\theta_1 [deg]'); grid on;
    subplot(4,1,4); plot(t_out, input, t_out, error); legend('input', 'error'); ylabel('Controller'); grid on;
    xlabel('Time [s]');
    sgtitle(sprintf('LQR'));