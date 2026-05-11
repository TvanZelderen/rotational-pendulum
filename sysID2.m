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
sim rotpentemplate
save_run(simin, simout, 'doublependulum_sine_excitation');

%% Extract states from Simulink output
y    = simout.Data;      % N x 5: [th1, dth1, th2, dth2, phi] in degrees

th1_rad  = deg2rad(y(:,1));
dth1_rad = deg2rad(y(:,2));
th2_rad  = deg2rad(y(:,3));
dth2_rad = deg2rad(y(:,4));

%% Check angles stay within +/- 20 deg
fprintf('Max |th1| = %.1f deg\n', max(abs(y(:,1))));
fprintf('Max |th2| = %.1f deg\n', max(abs(y(:,3))));
if max(abs(y(:,1))) > 20 || max(abs(y(:,3))) > 20
    warning('Angles exceed 20 deg — reduce A_exc!');
end

%% Overview plot — use to set trimStart / trimEnd
figure;
subplot(3,1,1); plot(t, y(:,1)); ylabel('\theta_1 [deg]'); xlabel('Time [s]');
subplot(3,1,2); plot(t, y(:,3)); ylabel('\theta_2 [deg]'); xlabel('Time [s]');
subplot(3,1,3); plot(t, u);      ylabel('u [V]');          xlabel('Time [s]');
sgtitle('Raw data — check angles and set trim bounds');

%% Trim transient (let system reach steady oscillation)
trimStart = 3;    % [s] skip startup transient
trimEnd   = 28;   % [s] cut end

iStart = round(trimStart / h) + 1;
iEnd   = round(trimEnd   / h) + 1;

y_trimmed = [th1_rad(iStart:iEnd),  dth1_rad(iStart:iEnd), ...
             th2_rad(iStart:iEnd),  dth2_rad(iStart:iEnd)];
u_trimmed = u(iStart:iEnd);
t_trimmed = t(iStart:iEnd);

%% iddata — 4 outputs, 1 input (voltage)
data = iddata(y_trimmed, u_trimmed, h);
data.OutputName = {'th1','dth1','th2','dth2'};
data.OutputUnit = {'rad','rad/s','rad','rad/s'};
data.InputName  = {'u'};
data.InputUnit  = {'V'};
data.Tstart     = t_trimmed(1);

%% Known / fixed values from hardware + previous single pendulum run
L1 = 0.10;    % [m]  link 1 length — measure from hardware
m2 = 0.10;    % [kg] link 2 mass   — weigh on scale

%% Initial parameter guesses
alpha1_0 = 0.05;              % fix this from single pendulum run if available
beta1_0  = 1.50;              % fix this from single pendulum run if available
alpha2_0 = 1e-3;
beta2_0  = 9.81 / 0.10;       % g/l2, l2 ~ 0.10 m initial guess
gamma_0  = m2 * L1 * 0.10;    % m2*L1*l2
km_0     = 0.10;               % motor constant [Nm/V] — tune to your motor
b2_0     = 0.01;               % link 2 damping

%% Build idnlgrey model — [ny, nu, nx] = [4, 1, 4]
sys0 = idnlgrey('doublependulum_ode', [4, 1, 4], ...
                {alpha1_0; beta1_0; alpha2_0; beta2_0; gamma_0; km_0; b2_0}, ...
                y_trimmed(1,:)', ...    % initial states from first data point
                0);                     % 0 = continuous time

%% Parameter settings
sys0.Parameters(1).Name    = 'alpha1';
sys0.Parameters(1).Minimum = 1e-4;
sys0.Parameters(1).Fixed   = true;    % known from single pendulum run

sys0.Parameters(2).Name    = 'beta1';
sys0.Parameters(2).Minimum = 0;
sys0.Parameters(2).Fixed   = true;    % known from single pendulum run

sys0.Parameters(3).Name    = 'alpha2';
sys0.Parameters(3).Minimum = 1e-4;
sys0.Parameters(3).Fixed   = false;

sys0.Parameters(4).Name    = 'beta2';
sys0.Parameters(4).Minimum = 0;
sys0.Parameters(4).Fixed   = false;

sys0.Parameters(5).Name    = 'gamma';
sys0.Parameters(5).Minimum = 1e-6;
sys0.Parameters(5).Fixed   = false;

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

%% Run estimation
sys_est = nlgreyest(data, sys0, opt);

%% Print results
fprintf('\n--- Estimated Parameters ---\n');
for i = 1:numel(sys_est.Parameters)
    fprintf('%-8s = %.6f\n', sys_est.Parameters(i).Name, sys_est.Parameters(i).Value);
end

%% Recover physical parameters
alpha2_est = sys_est.Parameters(3).Value;
beta2_est  = sys_est.Parameters(4).Value;
gamma_est  = sys_est.Parameters(5).Value;

l2_beta    = beta2_est / (m2 * 9.81);
l2_gamma   = gamma_est / (m2 * L1);
I2_est     = alpha2_est - m2 * l2_beta^2;

fprintf('\n--- Recovered Physical Parameters ---\n');
fprintf('l2 from beta2  = %.6f m\n', l2_beta);
fprintf('l2 from gamma  = %.6f m\n', l2_gamma);   % cross-check
fprintf('I2             = %.6f kg.m^2\n', I2_est);
fprintf('km             = %.6f Nm/V\n', sys_est.Parameters(6).Value);
fprintf('b2             = %.6f Nms/rad\n', sys_est.Parameters(7).Value);

%% Validate
figure; compare(data, sys_est);   % simulated vs measured
figure; resid(data, sys_est);     % residuals should look like white noise