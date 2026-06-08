%% -- Preamble --------------------------------------------

clear; clc;
calib;
hwinit;
pendulum_params;

h = 0.001;
controller_sat = 1;

% -- Stiction-breaking jiggle ----------------------------
% Mode 1 = off | 2 = continuous dither | 3 = conditional knocker
% Simulink reads these from the workspace via Constant / Gain blocks.
jiggle_mode = 1;

% Mode 1 — continuous dither (always-on, zero-mean)
A_d   = 0.03;          % amplitude [-]   start small
f_d   = 30;            % frequency [Hz]  above CL BW, below 500 Hz

% Mode 2 — conditional knocker (fires when arm 1 is stuck)
A_k   = 0.08;          % pulse amplitude [-]
f_k   = 30;            % buzz frequency [Hz]
eps_v = 0.08;          % |dth1_filt| < eps_v  → arm is stuck  [rad/s]
u_min = 0.05;          % |u_cmd| > u_min       → controller wants to move [-]

% Low-pass time constant for dth1 (noise filter for stuck gate)
tau_lp = 0.01;         % [s]  ~ 1/(2*pi*16 Hz) cutoff

% -- Controller type ----------------------------
% Mode 1 = Simin | 2 = LQR | 3 = MPC

controller_type = 3;

reference_indicator = 1; % 1 for down-down, 2 for down-up, 3 for up-up
if reference_indicator == 1
    ref = [0; 0; 0; 0]; % down-down
    offset = [0; 0];
    R = 50; % Tweak away
elseif reference_indicator == 2
    ref = [0; 0; pi; 0];
    offset = [0; -pi];
    R = 0.1; %25
elseif reference_indicator == 3
    ref = [-pi; 0; 0; 0];
    offset = [pi; 0];
    R = 100;
end

% Compute matrices LQR
[K, L, A, B, C] = compute_lqr(ref, R, p);

disp('Controller poles:'); disp(sort(eig(A - B*K), 'descend'))
disp('Observer poles:');   disp(sort(eig(A - L*C), 'descend'))

%Compute MPC object
[mpc_obj,Ad,Bd,Cd] = compute_mpc(ref, R, p);

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
phi = y(:,1) + y(:,3);
input  = y(:,6);

figure(1); clf;
subplot(3,2,1); plot(t_out, th1, t_out, oth1); yline(180,'k--');
  legend('\theta_1','\theta_1 obs'); ylabel('\theta_1 [deg]'); grid on;
subplot(3,2,2); plot(t_out, th2, t_out, oth2);
  legend('\theta_2','\theta_2 obs'); ylabel('\theta_2 [deg]'); grid on;
subplot(3,2,3); plot(t_out, dth1, t_out, odth1);
  legend('d\theta_1','d\theta_1 obs'); ylabel('d\theta_1'); grid on;
subplot(3,2,4); plot(t_out, dth2, t_out, odth2);
  legend('d\theta_2','d\theta_2 obs'); ylabel('d\theta_2'); grid on;
subplot(3,2,5); plot(t_out, input); yline(0.5,'r--'); yline(-0.5,'r--');
  ylabel('u'); xlabel('Time [s]'); grid on;
subplot(3,2,6); plot(t_out, e_vec);
  legend('e_{\theta1}','e_{d\theta1}','e_{\theta2}','e_{d\theta2}');
  ylabel('error [rad]'); xlabel('Time [s]'); grid on;
sgtitle('LQR');

e_th2 = y(:,13);
figure(2); clf;
yyaxis left;  plot(t_out, input);  ylabel('u [-]');
yyaxis right; plot(t_out, e_th2);  ylabel('e_{\theta2} [rad]');
xlabel('Time [s]'); grid on; legend('u','e_{\theta2}');

%% -- Jiggle diagnostics (cols 15-16; present only after Simulink wiring) --
% simout col 15 = jiggle [-]   col 16 = stuck [bool]
if size(y, 2) >= 16
    jiggle_log = y(:,15);
    stuck_log  = y(:,16);

    figure(3); clf;
    subplot(3,1,1);
    plot(t_out, input, 'k'); hold on;
    plot(t_out, jiggle_log, 'b--');
    yline(0.5,'r:'); yline(-0.5,'r:');
    ylabel('u, jiggle [-]'); legend('u_{total}','jiggle'); grid on;
    title(sprintf('Jiggle mode %d  (A_d=%.3f  A_k=%.3f  f=%.0f/%.0f Hz)', ...
          jiggle_mode, A_d, A_k, f_d, f_k));

    subplot(3,1,2);
    yyaxis left;  stairs(t_out, stuck_log); ylabel('stuck [-]'); ylim([-0.1 1.5]);
    yyaxis right; plot(t_out, rad2deg(y(:,2))); ylabel('d\theta_1 [deg/s]');
    grid on; legend('stuck','d\theta_1');

    subplot(3,1,3);
    plot(t_out, e_th2);
    ylabel('e_{\theta_2} [rad]'); xlabel('Time [s]'); grid on;
end

