clear; clc;

%% Parameters
h    = 0.01;
Tsim = 30;

t         = (0:h:Tsim)';
amplitude = 0.5;
omega     = 5;
u         = amplitude * sin(omega * t);
simin     = [t, u];

%% Load saved data
dataFolder = '/Users/vincentmaan/Master/rotational-pendulum/data';
fileName   = '20260511_165315_doublependulum_sine_excitation.mat';
runData    = load(fullfile(dataFolder, fileName));

u_raw  = runData.simin(:,2);
t_raw  = runData.simin(:,1);
y_raw  = runData.simout.Data;

th1_rad  = deg2rad(y_raw(:,1));
dth1_rad = deg2rad(y_raw(:,2));
th2_rad  = deg2rad(y_raw(:,3));
dth2_rad = deg2rad(y_raw(:,4));

%% Trim — full window, only exclude wrapping
trimStart = 3;
trimEnd   = 28;

iStart = round(trimStart/h) + 1;
iEnd   = round(trimEnd/h)   + 1;

wrap_mask = abs(th1_rad(iStart:iEnd)) < deg2rad(170) & ...
            abs(th2_rad(iStart:iEnd)) < deg2rad(170);

if sum(wrap_mask) < 500
    warning('Less than 5s of valid data!');
end

% Find calmest start within valid window
v_combined = abs(dth1_rad(iStart:iEnd)) + abs(dth2_rad(iStart:iEnd));
v_combined(~wrap_mask) = inf;
[~, iMin] = min(v_combined);
iStart    = iStart + iMin - 1;
iEnd      = round(trimEnd/h) + 1;

y_trimmed = [th1_rad(iStart:iEnd),  dth1_rad(iStart:iEnd), ...
             th2_rad(iStart:iEnd),  dth2_rad(iStart:iEnd)];
u_trimmed = u_raw(iStart:iEnd);
t_trimmed = t_raw(iStart:iEnd);

fprintf('Data window: t = %.2f to %.2f s (%d samples)\n', ...
        t_raw(iStart), t_raw(iEnd), iEnd-iStart+1);

%% iddata
data = iddata(y_trimmed, u_trimmed, h);
data.OutputName = {'th1','dth1','th2','dth2'};
data.OutputUnit = {'rad','rad/s','rad','rad/s'};
data.InputName  = {'u'};
data.InputUnit  = {'V'};
data.Tstart     = t_trimmed(1);

%% Known constants (fixed from hardware + single pendulum)
g  = 9.81;
L1 = 0.10;
m2 = 0.10;
alpha_damp = 0.325573;
beta_grav  = 98.729513;
l2 = g / beta_grav;              % 0.09936 m
I2 = 0.0;                        % point mass
b2 = alpha_damp * (m2 * l2^2);  % 3.21e-4 Nms/rad

fprintf('\n--- Fixed parameters (from single pendulum) ---\n')
fprintf('l2 = %.5f m\n', l2)
fprintf('I2 = %.5f kg.m^2\n', I2)
fprintf('b2 = %.2e Nms/rad\n', b2)

%% Initial guesses for estimated parameters (link 1 + motor)
m1_0 = 0.10;
l1_0 = 0.05;
I1_0 = 1e-3;    % give I1 a larger initial value to boost alpha1
km_0 = 0.05;
b1_0 = 0.01;                    % [Nms/rad]

%% Sanity check on denom
alpha1_check = I1_0 + m1_0*l1_0^2 + m2*L1^2;
alpha2_check = I2   + m2*l2^2;
gamma_check  = m2*L1*l2;
fprintf('\n--- Denom sanity check ---\n')
fprintf('alpha1*alpha2 = %.2e\n', alpha1_check*alpha2_check)
fprintf('gamma^2       = %.2e\n', gamma_check^2)
fprintf('ratio         = %.1f  (need > 10)\n', alpha1_check*alpha2_check / gamma_check^2)

%% Build idnlgrey — 5 estimated params: m1, l1, I1, km, b1
sys0 = idnlgrey('double_pen2', [4, 1, 4], ...
                {m1_0; l1_0; I1_0; km_0; b1_0}, ...
                y_trimmed(1,:)', ...
                0);

sys0.Parameters(1).Name    = 'm1';
sys0.Parameters(1).Minimum = 0.01;
sys0.Parameters(1).Maximum = 2.0;
sys0.Parameters(1).Fixed   = false;

sys0.Parameters(2).Name    = 'l1';
sys0.Parameters(2).Minimum = 0.001;
sys0.Parameters(2).Maximum = L1;
sys0.Parameters(2).Fixed   = false;

sys0.Parameters(3).Name    = 'I1';
sys0.Parameters(3).Minimum = 0;
sys0.Parameters(3).Maximum = 1.0;
sys0.Parameters(3).Fixed   = false;

sys0.Parameters(4).Name    = 'km';
sys0.Parameters(4).Minimum = 0;
sys0.Parameters(4).Maximum = 5.0;
sys0.Parameters(4).Fixed   = false;

sys0.Parameters(5).Name    = 'b1';
sys0.Parameters(5).Minimum = 0;
sys0.Parameters(5).Maximum = 1.0;
sys0.Parameters(5).Fixed   = false;

%% Initial states — all free
for i = 1:4
    sys0.InitialStates(i).Fixed = false;
end

%% Estimation options
opt = nlgreyestOptions('Display', 'on');
opt.SearchMethod                = 'lm';
opt.SearchOptions.MaxIterations = 300;
opt.SearchOptions.Tolerance     = 1e-8;

%% Diagnostic — test ODE directly before estimating
x0     = y_trimmed(1,:)';
u_test = u_trimmed(1);
[dx, ~] = double_pen2(0, x0, u_test, m1_0, l1_0, I1_0, km_0, b1_0);
fprintf('\n--- ODE diagnostic ---\n')
fprintf('dx = [%.4f, %.4f, %.4f, %.4f]\n', dx(1), dx(2), dx(3), dx(4))
if any(isnan(dx)) || any(isinf(dx))
    error('ODE returns NaN/Inf — check initial parameters!');
end

%% Quick test simulation
odefun = @(t,x) double_pen2(t, x, ...
    interp1(t_trimmed, u_trimmed, t, 'linear', 'extrap'), ...
    m1_0, l1_0, I1_0, km_0, b1_0);
odeOpts = odeset('RelTol',1e-6,'AbsTol',1e-8);
[t_test, x_test] = ode45(odefun, t_trimmed(1:min(200,length(t_trimmed))), x0, odeOpts);

figure;
subplot(2,1,1); plot(t_test, rad2deg(x_test(:,1)), t_test, rad2deg(x_test(:,3)));
legend('\theta_1','\theta_2'); ylabel('Angle [deg]');
title('ODE test simulation — check for blow-up');
subplot(2,1,2); plot(t_test, x_test(:,2), t_test, x_test(:,4));
legend('d\theta_1','d\theta_2'); ylabel('rad/s'); xlabel('Time [s]');

%% Run estimation
sys_est = nlgreyest(data, sys0, opt);

%% Results
fprintf('\n--- Estimated Parameters ---\n');
fprintf('m1  = %.6f kg\n',       sys_est.Parameters(1).Value);
fprintf('l1  = %.6f m\n',        sys_est.Parameters(2).Value);
fprintf('I1  = %.6f kg.m^2\n',   sys_est.Parameters(3).Value);
fprintf('km  = %.6f Nm/V\n',     sys_est.Parameters(4).Value);
fprintf('b1  = %.6f Nms/rad\n',  sys_est.Parameters(5).Value);

fprintf('\n--- Fixed from single pendulum ---\n');
fprintf('m2  = %.5f kg\n',       m2);
fprintf('l2  = %.5f m\n',        l2);
fprintf('I2  = %.5f kg.m^2\n',   I2);
fprintf('b2  = %.2e Nms/rad\n',  b2);

%% Validate
figure; compare(data, sys_est);
figure; resid(data, sys_est);