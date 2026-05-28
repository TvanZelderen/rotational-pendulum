% Closed-loop balance simulation — arm 2 upright.
% Hardware equivalent: rotpentemplate.slx with switch set to controller.

clear; clc;

pendulum_params;
linearise_upright;   % populates K, ref, A_num, B_num, C

use_lqr  = true;
h        = 0.001;    % [s]
Tsim     = 5;        % [s]
t        = (0 : h : Tsim)';
simin    = [t, zeros(size(t))];   % unused under use_lqr; kept for run_sim shape

%% -----------------------------------------------------------------------
%  Initial condition  (edit this line to test different perturbations)
% -----------------------------------------------------------------------
x0 = ref + [0; 0; 0.1; 0];   % small arm-2 angular perturbation [rad]

%% -----------------------------------------------------------------------
%  Run
% -----------------------------------------------------------------------
run_sim;

%% -----------------------------------------------------------------------
%  Extract clean states (pre-noise, pre-wrap — use x_out from run_sim)
% -----------------------------------------------------------------------
th1_clean = rad2deg(interp1(t_out, x_out(:,1), t));
th2_clean = rad2deg(interp1(t_out, x_out(:,3), t));

%% -----------------------------------------------------------------------
%  Plot
% -----------------------------------------------------------------------
figure(1); clf;
subplot(3,1,1);
plot(t, th2_clean, t, rad2deg(ref(3)) * ones(size(t)), 'k--');
legend('\theta_2', 'ref'); ylabel('[deg]'); grid on;
title('Balance controller — simulation');

subplot(3,1,2);
plot(t, th1_clean);
ylabel('\theta_1 [deg]'); grid on;

subplot(3,1,3);
plot(t, u_log);
yline(1, 'r--'); yline(-1, 'r--');
ylabel('u [-]'); xlabel('Time [s]'); grid on;

%% -----------------------------------------------------------------------
%  Animation
% -----------------------------------------------------------------------
animate_pendulum(simout, p);
