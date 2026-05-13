%clear; clc;

%% Parameters
h    = 0.01;   % sample period [s]
Tsim = 30;     % experiment duration [s]

%% Sine input signal — small angle excitation
f_exc = 2.0;   % excitation frequency [Hz] — tune to near resonance of link 1
A_exc = 1.5;   % amplitude [V]  — keep angles < 20 deg, tune on hardware

t     = (0:h:Tsim)';
amplitude = 0.5;
omega     = 5;
u = amplitude * sin(omega * t); 

% simin is read by the Simulink model: col 1 = time, col 2 = input
simin = [t, u];

%% Run experiment
%sim rotpentemplate
%save_run(simin, simout, 'doublependulum_sine_excitation');

%% saved measurement data

dataFolder = '/Users/vincentmaan/Master/rotational-pendulum/data';
fileName   = '20260511_165315_doublependulum_sine_excitation.mat'; % The file you want
% Load the file into a structure
runData = load(fullfile(dataFolder, fileName));

% Extract the original variables (assuming save_run used these names)
u_raw = runData.simin(:, 2);  % Column 2 of simin is the input voltage
t_raw     = runData.simin(:, 1);  % Column 1 is time
y_raw = runData.simout.Data;  % N x 5 matrix from the 'To Workspace' block

th1_rad  = deg2rad(y_raw(:,1));
dth1_rad = deg2rad(y_raw(:,2));
th2_rad  = deg2rad(y_raw(:,3));
dth2_rad = deg2rad(y_raw(:,4));

%% Extract states from Simulink output
%y    = simout.Data;      % N x 5: [th1, dth1, th2, dth2, phi] in degrees

%th1_rad  = deg2rad(y(:,1));
%dth1_rad = deg2rad(y(:,2));
%th2_rad  = deg2rad(y(:,3));
%dth2_rad = deg2rad(y(:,4));

%% Overview plot — use to set trimStart / trimEnd
%figure;
%subplot(3,1,1); plot(t, y(:,1)); ylabel('\theta_1 [deg]'); xlabel('Time [s]');
%subplot(3,1,2); plot(t, y(:,3)); ylabel('\theta_2 [deg]'); xlabel('Time [s]');
%subplot(3,1,3); plot(t, u);      ylabel('u [V]');          xlabel('Time [s]');
%sgtitle('Raw data — check angles and set trim bounds');

%% Trim transient (let system reach steady oscillation)
trimStart = 3;    % [s] skip startup transient
trimEnd   = 28;   % [s] cut end

%iStart = round(trimStart / h) + 1;
%iEnd   = round(trimEnd   / h) + 1;

% Pick a calmer starting point — find where velocity is near zero
%[~, iStart] = min(abs(dth2_rad(round(trimStart/h):round(trimEnd/h))));
%iStart = iStart + round(trimStart/h);

%y_trimmed = [th1_rad(iStart:iEnd),  dth1_rad(iStart:iEnd), ...
%             th2_rad(iStart:iEnd),  dth2_rad(iStart:iEnd)];

%% Find starting index where BOTH velocities are small

%% Trim — use full window, no angle restriction
iStart = round(trimStart/h) + 1;
iEnd   = round(trimEnd/h)   + 1;

% Only filter out wrapping (angles beyond +/-170 deg)
wrap_mask = abs(th1_rad(iStart:iEnd)) < deg2rad(170) & ...
            abs(th2_rad(iStart:iEnd)) < deg2rad(170);

if sum(wrap_mask) < 500
    warning('Less than 5s of valid data!');
end

% Find calmest start (low velocity) within valid window
v_combined = abs(dth1_rad(iStart:iEnd)) + abs(dth2_rad(iStart:iEnd));
v_combined(~wrap_mask) = inf;
[~, iMin]  = min(v_combined);
iStart     = iStart + iMin - 1;
iEnd       = round(trimEnd/h) + 1;

fprintf('Data window: t = %.2f to %.2f s (%d samples)\n', ...
        t_raw(iStart), t_raw(iEnd), iEnd - iStart + 1);

fprintf('Trim start: t = %.2f s, th1 = %.1f deg, dth1 = %.3f rad/s, dth2 = %.3f rad/s\n', ...
        t_raw(iStart), rad2deg(th1_rad(iStart)), dth1_rad(iStart), dth2_rad(iStart));

y_trimmed = [th1_rad(iStart:iEnd),  dth1_rad(iStart:iEnd), ...
             th2_rad(iStart:iEnd),  dth2_rad(iStart:iEnd)];

%CHANGE RAW IF USING REAL SETUP!!!!!!!!!!!!!!!!!!!

u_trimmed = u_raw(iStart:iEnd);
t_trimmed = t_raw(iStart:iEnd);

%% iddata — 4 outputs, 1 input (voltage)
data = iddata(y_trimmed, u_trimmed, h);
data.OutputName = {'th1','dth1','th2','dth2'};
data.OutputUnit = {'rad','rad/s','rad','rad/s'};
data.InputName  = {'u'};
data.InputUnit  = {'V'};
data.Tstart     = t_trimmed(1);

%% Recover from single pendulum results
alpha_damp = 0.325573;   % c2 / (I2 + m2*l2^2)   [1/s]
beta_grav  = 98.729513;  % m2*l2*g / (I2 + m2*l2^2)  [rad/s^2]

m2 = 0.10;   % [kg]
L1 = 0.10;   % [m]
g  = 9.81;

% From beta: l2 = (I2+m2*l2^2)*beta / (m2*g)
% But we don't know I2 yet. If we assume I2 << m2*l2^2 (point mass approx):
l2_est    = g / beta_grav;               % = 0.09936 m  (same as before)

% Now inertia:
alpha2_0  = m2 * l2_est^2;              % I2 + m2*l2^2 ≈ m2*l2^2 (point mass)
beta2_0   = m2 * l2_est * g;            % m2*l2*g
gamma_0   = m2 * L1 * l2_est;

% Damping on link 2 recovered from alpha_damp:
b2_0      = alpha_damp * alpha2_0;      % c2 = alpha_damp * (I2 + m2*l2^2)

%% Initial parameter guesses            

%Motor arm: treat as uniform rod, mass m1 ~ 0.1 kg, length L1 = 0.1 m
m1_guess = 0.4;
I1_guess = (1/3) * m1_guess * L1^2;   % uniform rod about end
l1_guess = L1 / 2;                     % CoM at midpoint

alpha1_0 = I1_guess + m1_guess*l1_guess^2 + m2*L1^2;
beta1_0  = (m1_guess*l1_guess + m2*L1) * g;
km_0     = 0.05;               % motor constant [Nm/V] — tune to your motor

fprintf('\n--- Sanity check ---\n')
fprintf('alpha2_0 = %.2e  (expect ~1e-3)\n', alpha2_0)
fprintf('beta2_0  = %.4f  (expect ~0.097)\n', beta2_0)
fprintf('gamma_0  = %.2e  (expect ~1e-4)\n', gamma_0)
fprintf('alpha1_0 = %.2e  (expect ~1e-3)\n', alpha1_0)
fprintf('beta1_0  = %.4f  (expect ~0.1-1.0)\n', beta1_0)
fprintf('denom check: alpha1*alpha2=%.2e, gamma^2=%.2e, ratio=%.1f\n', ...
        alpha1_0*alpha2_0, gamma_0^2, alpha1_0*alpha2_0/gamma_0^2)

%% Diagnostic plot of trimmed data
figure;
subplot(3,1,1); plot(t_trimmed, rad2deg(y_trimmed(:,1))); 
    yline(15,'r--'); yline(-15,'r--');
    ylabel('\theta_1 [deg]'); title(sprintf('Trimmed window: %.2f to %.2f s', t_trimmed(1), t_trimmed(end)));
subplot(3,1,2); plot(t_trimmed, rad2deg(y_trimmed(:,3)));
    yline(15,'r--'); yline(-15,'r--');
    ylabel('\theta_2 [deg]');
subplot(3,1,3); plot(t_trimmed, u_trimmed);
    ylabel('u [V]'); xlabel('Time [s]');
fprintf('Data window length: %.1f s (%d samples)\n', ...
        t_trimmed(end)-t_trimmed(1), length(t_trimmed));
fprintf('Need at least 5-10s for reliable identification.\n');

%% Build idnlgrey model — [ny, nu, nx] = [4, 1, 4]
sys0 = idnlgrey('double_pen', [4, 1, 4], ...
                {alpha1_0; beta1_0; alpha2_0; beta2_0; gamma_0; km_0; b2_0}, ...
                y_trimmed(1,:)', ...
                0);                     % 0 = continuous time

%% Parameter settings
sys0.Parameters(1).Name    = 'alpha1';
sys0.Parameters(1).Minimum = 1e-4;
sys0.Parameters(1).Fixed   = false;    % known from single pendulum run

sys0.Parameters(2).Name    = 'beta1';
sys0.Parameters(2).Minimum = 0;
sys0.Parameters(2).Fixed   = false;    % known from single pendulum run

sys0.Parameters(3).Name    = 'alpha2';
sys0.Parameters(3).Minimum = 1e-4;
sys0.Parameters(3).Fixed   = true;

sys0.Parameters(4).Name    = 'beta2';
sys0.Parameters(4).Minimum = 0;
sys0.Parameters(4).Fixed   = true;

sys0.Parameters(5).Name    = 'gamma';
sys0.Parameters(5).Minimum = 1e-6;
sys0.Parameters(5).Fixed   = true;

sys0.Parameters(6).Name    = 'km';
sys0.Parameters(6).Minimum = 0;
sys0.Parameters(6).Fixed   = false;

sys0.Parameters(7).Name    = 'b2';
sys0.Parameters(7).Minimum = 0;
sys0.Parameters(7).Fixed   = false;

%% Initial states — all free (hard to know exact starting velocities)
for i = 1:4
    sys0.InitialStates(i).Fixed = false;
end

%% Estimation options
opt = nlgreyestOptions('Display', 'on');
opt.SearchMethod                  = 'lm';
opt.SearchOptions.MaxIterations   = 300;
opt.SearchOptions.Tolerance       = 1e-8;

%% --- DIAGNOSTIC: test ODE at initial conditions ---
alpha1 = alpha1_0;
beta1  = beta1_0;
alpha2 = alpha2_0;
beta2  = beta2_0;
gamma  = gamma_0;
km     = km_0;
b2     = b2_0;

x0    = y_trimmed(1,:)';
u_test = u_trimmed(1);

fprintf('--- Initial states ---\n');
fprintf('th1  = %.4f rad (%.2f deg)\n', x0(1), rad2deg(x0(1)));
fprintf('dth1 = %.4f rad/s\n', x0(2));
fprintf('th2  = %.4f rad (%.2f deg)\n', x0(3), rad2deg(x0(3)));
fprintf('dth2 = %.4f rad/s\n', x0(4));

fprintf('\n--- Initial parameters ---\n');
fprintf('alpha1 = %.6f\n', alpha1);
fprintf('beta1  = %.6f\n', beta1);
fprintf('alpha2 = %.6f\n', alpha2);
fprintf('beta2  = %.6f\n', beta2);
fprintf('gamma  = %.6f\n', gamma);
fprintf('km     = %.6f\n', km);
fprintf('b2     = %.6f\n', b2);

% Check denominator
D     = x0(1) - x0(3);
denom = alpha1*alpha2 - gamma^2*cos(D)^2;
fprintf('\n--- Denominator check ---\n');
fprintf('D (th1-th2) = %.4f rad\n', D);
fprintf('alpha1*alpha2       = %.6f\n', alpha1*alpha2);
fprintf('gamma^2*cos(D)^2    = %.6f\n', gamma^2*cos(D)^2);
fprintf('denom               = %.6f  <-- must be nonzero!\n', denom);

% Try calling the ODE directly
[dx, yout] = double_pen(0, x0, u_test, alpha1, beta1, alpha2, beta2, gamma, km, b2);
fprintf('\n--- ODE output ---\n');
fprintf('dx = [%.4f, %.4f, %.4f, %.4f]\n', dx(1), dx(2), dx(3), dx(4));

% Quick simulation
odefun = @(t,x) double_pen(t, x, interp1(t_trimmed, u_trimmed, t, 'linear', 'extrap'), ...
                            alpha1, beta1, alpha2, beta2, gamma, km, b2);
opts = odeset('RelTol',1e-6,'AbsTol',1e-8);
[t_test, x_test] = ode45(odefun, t_trimmed(1:min(100, length(t_trimmed))), x0, opts);

figure;
subplot(2,1,1); plot(t_test, rad2deg(x_test(:,1)), t_test, rad2deg(x_test(:,3)));
legend('\theta_1','\theta_2'); ylabel('deg'); title('ODE test simulation (first 1s)');
subplot(2,1,2); plot(t_test, x_test(:,2), t_test, x_test(:,4));
legend('d\theta_1','d\theta_2'); ylabel('rad/s'); xlabel('Time [s]');
%% Run estimation
sys_est = nlgreyest(data, sys0, opt);

%% Print results
fprintf('\n--- Estimated Parameters ---\n');
for i = 1:numel(sys_est.Parameters)
    fprintf('%-8s = %.6f\n', sys_est.Parameters(i).Name, sys_est.Parameters(i).Value);
end

%% Recover physical parameters
% Link 2 — already known from single pendulum (fixed params, just report)
l2        = g / beta_grav;                  % from single pendulum
I2        = alpha2_0 - m2 * l2^2;          % from single pendulum
% Link 1 — newly estimated
alpha1_est = sys_est.Parameters(1).Value;
beta1_est  = sys_est.Parameters(2).Value;
km_est     = sys_est.Parameters(6).Value;
b2_est     = sys_est.Parameters(7).Value;

% Recover link 1 physical params
% beta1 = (m1*l1 + m2*L1)*g  → m1*l1 = beta1/g - m2*L1
m1l1_est  = beta1_est / g - m2 * L1;

% alpha1 = I1 + m1*l1^2 + m2*L1^2  → I1 = alpha1 - m1*l1^2 - m2*L1^2
I1_est    = alpha1_est - m1l1_est^2/m1_guess - m2*L1^2;  % needs m1 assumption

fprintf('\n--- Fixed from single pendulum ---\n');
fprintf('l2             = %.6f m\n',       l2);
fprintf('I2             = %.6f kg.m^2\n',  I2);

fprintf('\n--- Estimated from double pendulum ---\n');
fprintf('alpha1         = %.6f kg.m^2\n',  alpha1_est);
fprintf('beta1          = %.6f Nm\n',      beta1_est);
fprintf('km             = %.6f Nm/V\n',    km_est);
fprintf('b2             = %.6f Nms/rad\n', b2_est);

fprintf('\n--- Recovered link 1 physical params (needs m1 assumption) ---\n');
fprintf('m1*l1          = %.6f kg.m\n',    m1l1_est);
fprintf('I1 (approx)    = %.6f kg.m^2\n',  I1_est);

%% Validate
%figure; compare(data, sys_est);   % simulated vs measured
%figure; resid(data, sys_est);     % residuals should look like white noise