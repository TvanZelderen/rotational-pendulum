%% - Setup and Linearize ---------------------------------
% -- Preamble --------------------------------------------

clear; clc;
calib;
hwinit;
pendulum_params;

% -- Config ----------------------------------------------

h = 0.001;
controller_sat = 1;

% -- Controller type ----------------------------
% 1 = Simin | 2 = LQR | 3 = MPC
controller_type = 2;

% -- Reference point ----------------------------
% 1 = down-down | 2 = down-up | 3 = up-up
reference_indicator = 3;

% -- LQR options (controller_type == 2 only) ----
use_lqi = 1;           % 0 = pure LQR | 1 = LQR + integral action on θ₁

% -- DEMO (controller_type == 2 only) -----------
% 0 = hold | 1 = step | 2 = sine | 3 = waypoints
DEMO = 5;

% -- Run duration (MPC default; LQR overrides per DEMO) --
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

    % -- DEMO trajectory ----------------------------------
    %
    % Bring-up order (read before first powered run):
    %   1. DEMO 0, relay off / arm held — confirm logging, u stays bounded.
    %   2. DEMO 0, powered, pendulum balanced by hand — confirm balance holds.
    %   3. DEMO 1 (step) — small move; watch u, watch θ₂ near equilibrium.
    %   4. DEMO 2, 3. Raise amplitude only after each clean run.

    switch DEMO
        case 0
            run_time = 8;
            ref_fn   = @(t) 0;
            ttl      = 'Hold: θ₁ = 0°';
        case 1
            run_time = 12;
            ref_fn   = @(t) deg2rad(30) * (t >= 3);
            ttl      = 'Step: arm 1 +30° at t = 3 s';
        case 2
            run_time = 12;
            ref_fn   = @(t) deg2rad(45) * sin(2*pi * 0.25 * t);
            ttl      = 'Sine sweep: ±45°, 0.25 Hz';
        case 3
            run_time = 13;
            ref_fn   = @(t) deg2rad(35)*(t >= 2 & t < 5) + deg2rad(-35)*(t >= 5 & t < 8);
            ttl      = 'Waypoints: 0° → +35° → −35° → 0°';
        case 4
            run_time = 12;
            ref_fn   = @(t) deg2rad(30) * sin(2*pi * 0.15 * t);
            ttl      = 'Sine sweep: ±45°, 0.25 Hz';
        case 5
            run_time = 20;
            ref_fn   = @(t) deg2rad(30)*(t >= 5 & t < 10) + deg2rad(-30)*(t >= 10 & t < 15);
            ttl      = 'Waypoints: 0° → +35° → −35° → 0°';
    end

    % -- Reference timeseries (From Workspace payload) ----
    %
    %  Model works in deviation coords: x̂ = x − x_eq → equilibrium = [0;0;0;0].
    %  ref_x in deviation coords: [δ; 0; −δ; 0].
    %
    %  Psi-coupling: θ₁_dev = +δ → θ₂_dev = −δ keeps inertial arm-2 angle
    %  (θ₁+θ₂) fixed at ref(1)+ref(3) (e.g. π for down-up).

    t_vec       = (0 : h : run_time)';
    th1_ref_dev = arrayfun(ref_fn, t_vec);   % δ(t) — deviation from equilibrium
    th2_ref_dev = -th1_ref_dev;              % −δ(t) — psi coupling

    ref_x       = timeseries([th1_ref_dev, zeros(size(t_vec)), th2_ref_dev, zeros(size(t_vec))], t_vec);
    ref_x.Name  = 'ref_x';

elseif controller_type == 3
    use_lqi = 0;
    [K_full, L, A, B, C] = compute_lqr(ref, R, p, 0);
    K_int = 0;
    K = K_full;
    [mpc_obj, Ad, Bd, Cd] = compute_mpc(ref, R, p, reference_indicator);

else
    error('controller_type %d not implemented', controller_type)
end

%% -- Run -------------------------------------------------

ctrl_names = {'simin', 'lqr', 'mpc'};
ref_names  = {'down-down', 'down-up', 'up-up'};

if controller_type == 2
    run_name = sprintf('%s_%s_demo%d', ctrl_names{controller_type}, ref_names{reference_indicator}, DEMO);
    sim rotpen_lqr;
elseif controller_type == 3
    run_name = sprintf('%s_%s', ctrl_names{controller_type}, ref_names{reference_indicator});
    sim rotpen_mpc;
end

save_run(0, simout, run_name);
fprintf('Saved: %s\n', run_name);

%% -- Plot ------------------------------------------------

if controller_type == 2
    t_out = simout.Time;
    y     = simout.Data;
    th1   = y(:,1);    % [deg]
    th2   = y(:,3);    % [deg]
    u     = y(:,6);    % [-]

    % Convert deviation refs to absolute degrees for overlay on hardware output
    th1_ref_deg = rad2deg(ref(1)) + rad2deg(th1_ref_dev);
    th2_ref_deg = rad2deg(ref(3)) + rad2deg(th2_ref_dev);

    figure(1); clf;

    subplot(3,1,1);
    plot(t_out, th1, 'b', t_vec, th1_ref_deg, 'k--', 'LineWidth', 1.2);
    ylabel('\theta_1 [deg]'); legend('actual', 'reference'); grid on;
    title(['Reference tracking — ' ttl]);

    subplot(3,1,2);
    plot(t_out, th2, 'r', t_vec, th2_ref_deg, 'k--', 'LineWidth', 1.2);
    yline(rad2deg(ref(3)), 'k:');
    ylabel('\theta_2 [deg]'); legend('actual', 'reference'); grid on;

    subplot(3,1,3);
    plot(t_out, u, 'k'); yline(controller_sat, 'r--'); yline(-controller_sat, 'r--');
    ylabel('u [-]'); xlabel('Time [s]'); grid on;

    sgtitle(ttl);

else
    fprintf('Plot with: plot_lqr\n');
end
