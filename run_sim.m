% Simulation runner — drop-in replacement for running rotpentemplate.slx
% without hardware.  Produces the same workspace variables as firstScript.m:
%   simout  — Timeseries with columns [theta1_deg, theta2_deg]
%
% Do not call this directly — it is invoked by firstScript.m when
% run_with_simulation = true.
%
% Prerequisites:
%   pendulum_params.m   — must have been run (populates struct p)
%   rotpen_ode.m        — must have your EOM filled in

%% -----------------------------------------------------------------------
%  Initial conditions
%  theta1 = 0 : arm 1 hanging straight down
%  theta2 = 0 : arm 2 aligned with downward extension of arm 1
% -----------------------------------------------------------------------
th1_0  = 0;      % [rad]
dth1_0 = 0;      % [rad/s]
th2_0  = 0.5;    % [rad]  small push away from equilibrium
dth2_0 = 0;      % [rad/s]

x0 = [th1_0; dth1_0; th2_0; dth2_0];

%% -----------------------------------------------------------------------
%  Input signal
%  simin: col 1 = time [s],  col 2 = voltage command [V]
%  If simin already exists in the workspace (set by a controller script),
%  this block is skipped.
% -----------------------------------------------------------------------
if ~exist('simin', 'var')
    h    = 0.01;
    Tsim = 10;
    t    = (0 : h : Tsim)';
    simin = [t, zeros(size(t))];
end

t_span = simin(:, 1);
u_cmd  = simin(:, 2);

%% -----------------------------------------------------------------------
%  Integrate the ODE
% -----------------------------------------------------------------------
u_interp = @(t) interp1(t_span, u_cmd, t, 'linear', u_cmd(end));
odefun   = @(t, x) rotpen_ode(t, x, u_interp(t), p);

opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
[t_out, x_out] = ode45(odefun, [t_span(1), t_span(end)], x0, opts);

%% -----------------------------------------------------------------------
%  Resample onto the fixed grid and convert to degrees
% -----------------------------------------------------------------------
th1_deg  = rad2deg(interp1(t_out, x_out(:,1), t_span));
th2_deg  = rad2deg(interp1(t_out, x_out(:,3), t_span));

%% -----------------------------------------------------------------------
%  Add sensor bias and noise  (grey-box: match real hardware output format)
%
%  The hardware setup has constant offsets on both sensors, normally removed
%  by a Simulink constant block.  We add them here so simulated data is in
%  the same "raw" format.  Bias values live in pendulum_params.m.
%
%  Gaussian noise is added to mimic encoder/ADC measurement noise.
%  Noise std devs also live in pendulum_params.m.
% -----------------------------------------------------------------------
N = length(t_span);

th1_deg = th1_deg + p.bias_th1 + p.noise_std_th1 * randn(N, 1);
th2_deg = th2_deg + p.bias_th2 + p.noise_std_th2 * randn(N, 1);
psi_deg = th1_deg + th2_deg;

% Differentiate noisy angle signals to match Simulink derivative block (c=inf)
h_sim    = mean(diff(t_span));
dth1_dps = [0; diff(th1_deg)] / h_sim;
dth2_dps = [0; diff(th2_deg)] / h_sim;

%% -----------------------------------------------------------------------
%  Pack into simout Timeseries  (matches rotpentemplate.slx output format)
%  Columns: [th1_deg, dth1_deg/s, th2_deg, dth2_deg/s, psi_deg]
% -----------------------------------------------------------------------
simout = timeseries([th1_deg, dth1_dps, th2_deg, dth2_dps, psi_deg], t_span);
simout.Name = 'simout';

disp('run_sim: done.  simout is in the workspace.');
