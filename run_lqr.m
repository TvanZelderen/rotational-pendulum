%% -- Preamble --------------------------------------------

clear; clc;
calib;
hwinit;
pendulum_params;

h = 0.001;
R = 1;
controller_sat = 0.5;
% ref = [0; 0; 0; 0]; % down-down
% ref = [0; 0; pi; 0]; % down-up
ref = [pi; 0; 0; 0]; % up-up

[K, L, A, B, C] = compute_lqr(ref, R, p);

disp('Controller poles:'); disp(sort(eig(A - B*K), 'descend'))
disp('Observer poles:');   disp(sort(eig(A - L*C), 'descend'))

run_time = 20;

% Controller poles:
%  -76.8432 + 0.0000i
%   -0.6431 +10.5164i
%   -0.6431 -10.5164i
%   -1.0083 + 0.0000i
% 
% Observer poles:
%    -5.9341
%   -26.3648
%   -32.3341
%   -72.5983

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
e_vec = y(:,11:14);   % controller error ref - x, all 4 states (rad)

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

