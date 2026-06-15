%% - Setup and Linearize ---------------------------------
% -- Preamble --------------------------------------------

clear; clc;
calib;
hwinit;
pendulum_params;

% -- Config ----------------------------------------------

h = 0.001;
controller_sat = 1;

% -- Stiction-breaking jiggle ----------------------------
% Mode 1 = off | Mode 2 = continuous dither | Mode 3 = conditional knocker
% Simulink reads these from the workspace via Constant / Gain blocks.
jiggle_mode = 1;

% Mode 2 — continuous dither (always-on, zero-mean)
A_d   = 0.03;          % amplitude [-]   start small
f_d   = 30;            % frequency [Hz]  above CL BW, below 500 Hz

% Mode 3 — conditional knocker (fires when arm 1 is stuck)
A_k   = 0.08;          % pulse amplitude [-]
f_k   = 30;            % buzz frequency [Hz]
eps_v = 0.08;          % |dth1_filt| < eps_v  → arm is stuck  [rad/s]
u_min = 0.05;          % |u_cmd| > u_min       → controller wants to move [-]

% Low-pass time constant for dth1 (noise filter for stuck gate)
tau_lp = 0.01;         % [s]  ~ 1/(2*pi*16 Hz) cutoff

% -- Controller type ----------------------------
% 1 = Simin | 2 = LQR | 3 = MPC
controller_type = 2;

% -- Reference point ----------------------------
% 1 = down-down | 2 = down-up | 3 = up-up
reference_indicator = 3;

% -- LQR options (controller_type == 2 only) ----
use_lqi = 1;           % 0 = pure LQR | 1 = LQR + integral action on θ₁

% -- Run duration --------------------------------
run_time = 20;

% -- Reference setup -------------------------------------

if reference_indicator == 1
    ref = [0; 0; 0; 0]; % down-down
    offset = [0; 0];
    R = 50;
elseif reference_indicator == 2
    ref = [0; 0; pi; 0];
    offset = [0; -pi];
    R = 25;
elseif reference_indicator == 3
    ref = [-pi; 0; 0; 0];
    offset = [pi; 0];
    R = 200;
end

% -- Controller build ------------------------------------

if controller_type == 2
    [K_full, L, A, B, C] = compute_lqr(ref, R, p, use_lqi);

    if use_lqi
        K_int = K_full(5);          % integral gain on θ₁ — read by Simulink
        K     = K_full(1:4);        % state feedback gain
        A_aug = [A, zeros(4,1); [1,0,0,0], 0];
        B_aug = [B; 0];
        disp('Controller poles (LQI, 5-state):');
        disp(sort(real(eig(A_aug - B_aug*K_full)), 'descend'))
    else
        K_int = 0;
        K     = K_full;
        disp('Controller poles:'); disp(sort(eig(A - B*K), 'descend'))
    end
    disp('Observer poles:'); disp(sort(eig(A - L*C), 'descend'))

elseif controller_type == 3
    use_lqi = 0;
    [K_full, L, A, B, C] = compute_lqr(ref, R, p, 0);
    K_int = 0;
    K = K_full;
    [mpc_obj, Ad, Bd, Cd] = compute_mpc(ref, R, p);

else
    error('controller_type %d not implemented', controller_type)
end

%% -- Run -------------------------------------------------

ctrl_names = {'simin', 'lqr', 'mpc'};
ref_names  = {'down-down', 'down-up', 'up-up'};
run_name   = sprintf('%s_%s', ctrl_names{controller_type}, ref_names{reference_indicator});

if controller_type == 2
    % LQR     
    sim rotpen_lqr;
elseif controller_type == 3
    % MPC
    sim rotpen_mpc;
end
save_run(0, simout, run_name);

fprintf('Saved: %s\n', run_name);
fprintf('Plot with: plot_lqr\n');
