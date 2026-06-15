%% sim_mpc_balance.m
%  Validate the linear MPC as a BALANCE (catch) controller for down-up.
%
%  Two back-to-back runs, same mpc_obj:
%    Run A — plant starts 10 deg off down-up  → should catch and hold
%    Run B — plant starts at down-down         → should fail (confirms diagnosis)
%
%  No Simulink, no observer, no offset wiring. Pure mpcmove + RK4.
%  Uses the production compute_mpc so the actual controller object is tested.
%
%  Key correctness note:
%    compute_mpc sets Model.Nominal.Y = C*x0 = [0; pi] (nonzero!).
%    mpcmove expects ABSOLUTE ym and r; it subtracts Nominal.Y internally.
%    => pass ym = C*x (absolute), r = C*x0 (absolute).
%    Do NOT copy test.m's y_dev = C*x - C*x0 pattern: it only works
%    there because Nominal.Y = 0 at down-down. Here it would double-subtract.

clear; clc;
warning off backtrace;
diary off;
pendulum_params;  % loads p

%% ---- Operating point & MPC build ------------------------------------------
x0 = [0; 0; pi; 0];   % down-up: arm1 hanging, arm2 relative-up
R  = 10;               % from run_mpc.m for down-up (currently unused inside
                       % compute_mpc, but kept for future wiring)

fprintf('Building MPC object (symbolic Jacobian — takes ~30 s) ...\n');
tic;
[mpc_obj, Ad, Bd, Cd] = compute_mpc(x0, R, p);
fprintf('Done in %.1f s\n\n', toc);

%  C from compute_mpc:
%    row1 = [1 0 0 0]   → th1
%    row2 = [1 0 1 0]   → psi = th1 + th2  (arm-2 inertial angle)
C = Cd;

Ts    = 0.001;
t_end = 5.0;
n     = round(t_end / Ts);
t     = (0:n-1) * Ts;

%% ======================================================================
%% RUN A — catch: start 10 deg off down-up
%% ======================================================================
fprintf('=== Run A: catch (start 10 deg off down-up) ===\n');

x_A     = x0 + [0; 0; deg2rad(10); 0];
xlog_A  = zeros(4, n);
ulog_A  = zeros(1, n);
st_A    = mpcstate(mpc_obj);   % internal state initialised at Nominal.X = x0

for k = 1:n
    xlog_A(:,k) = x_A;

    % Absolute measurement and reference — MPC subtracts Nominal.Y internally
    ym = C * x_A;   % [th1; psi]   absolute
    r  = C * x0;    % [0;  pi]     absolute set-point

    [u, info] = mpcmove(mpc_obj, st_A, ym, r);
    if info.QPCode ~= 1
        fprintf('  k=%d  QP infeasible (code %d)\n', k, info.QPCode);
    end
    u = max(-1, min(1, u));   % hard clamp (safety; MPC already constrained)
    ulog_A(k) = u;

    % RK4 on true nonlinear ODE
    k1 = rotpen_ode(0, x_A,             u, p);
    k2 = rotpen_ode(0, x_A + Ts/2*k1,  u, p);
    k3 = rotpen_ode(0, x_A + Ts/2*k2,  u, p);
    k4 = rotpen_ode(0, x_A + Ts*k3,    u, p);
    x_A = x_A + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4);
end

th2_A = rad2deg(xlog_A(3,:));
psi_A = rad2deg(xlog_A(1,:) + xlog_A(3,:));  % arm-2 inertial angle
fprintf('  Final th2  = %.2f deg  (target 180 deg)\n', th2_A(end));
fprintf('  Final psi  = %.2f deg  (target 180 deg)\n', psi_A(end));
fprintf('  Max |u|    = %.4f\n\n', max(abs(ulog_A)));

%% ======================================================================
%% RUN B — diagnosis: start at down-down (pi away from operating point)
%% ======================================================================
fprintf('=== Run B: diagnosis (start at down-down) ===\n');

x_B    = [0; 0; 0; 0];
xlog_B = zeros(4, n);
ulog_B = zeros(1, n);
st_B   = mpcstate(mpc_obj);

for k = 1:n
    xlog_B(:,k) = x_B;

    ym = C * x_B;
    r  = C * x0;

    [u, ~] = mpcmove(mpc_obj, st_B, ym, r);
    u = max(-1, min(1, u));
    ulog_B(k) = u;

    k1 = rotpen_ode(0, x_B,             u, p);
    k2 = rotpen_ode(0, x_B + Ts/2*k1,  u, p);
    k3 = rotpen_ode(0, x_B + Ts/2*k2,  u, p);
    k4 = rotpen_ode(0, x_B + Ts*k3,    u, p);
    x_B = x_B + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4);
end

th2_B = rad2deg(xlog_B(3,:));
psi_B = rad2deg(xlog_B(1,:) + xlog_B(3,:));
fprintf('  Initial u  = %.4f  (expect +-1)\n', ulog_B(1));
fprintf('  Max |u|    = %.4f\n\n', max(abs(ulog_B)));

%% ======================================================================
%% PLOTS
%% ======================================================================
figure(10); clf;
sgtitle('MPC down-up balance validation');

%-- Row 1: psi (arm-2 inertial angle, target 180 deg)
subplot(3,2,1);
plot(t, psi_A); yline(180,'r--','LineWidth',1.2);
ylabel('\psi = \theta_1+\theta_2 [deg]'); title('Run A — catch (start 10° off)'); grid on;

subplot(3,2,2);
plot(t, psi_B, 'Color', [0.8 0.2 0.2]); yline(180,'r--','LineWidth',1.2);
ylabel('\psi [deg]'); title('Run B — diagnosis (start at down-down)'); grid on;

%-- Row 2: th2 relative
subplot(3,2,3);
plot(t, th2_A); yline(180,'r--','LineWidth',1.2);
ylabel('\theta_2 [deg]'); grid on;

subplot(3,2,4);
plot(t, th2_B, 'Color', [0.8 0.2 0.2]); yline(180,'r--','LineWidth',1.2);
ylabel('\theta_2 [deg]'); grid on;

%-- Row 3: control input
subplot(3,2,5);
plot(t, ulog_A); yline(1,'k--'); yline(-1,'k--');
ylabel('u [-]'); xlabel('Time [s]'); grid on; ylim([-1.1 1.1]);

subplot(3,2,6);
plot(t, ulog_B, 'Color', [0.8 0.2 0.2]); yline(1,'k--'); yline(-1,'k--');
ylabel('u [-]'); xlabel('Time [s]'); grid on; ylim([-1.1 1.1]);

%% ======================================================================
%% REGION OF ATTRACTION SWEEP (optional — comment out if slow)
%% ======================================================================
fprintf('=== Sweep: region of attraction (initial th2 offset) ===\n');
offsets_deg = [5 10 20 30 45 60 90 120];
converged   = false(size(offsets_deg));

for j = 1:numel(offsets_deg)
    x_s   = x0 + [0; 0; deg2rad(offsets_deg(j)); 0];
    st_s  = mpcstate(mpc_obj);
    for k = 1:n
        ym = C * x_s;  r = C * x0;
        [u, ~] = mpcmove(mpc_obj, st_s, ym, r);
        u = max(-1, min(1, u));
        k1 = rotpen_ode(0, x_s,             u, p);
        k2 = rotpen_ode(0, x_s + Ts/2*k1,  u, p);
        k3 = rotpen_ode(0, x_s + Ts/2*k2,  u, p);
        k4 = rotpen_ode(0, x_s + Ts*k3,    u, p);
        x_s = x_s + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4);
    end
    % Converged if final th2 within 10 deg of 180
    converged(j) = abs(rad2deg(x_s(3)) - 180) < 10;
    fprintf('  offset %3d deg  →  final th2 = %6.1f deg  [%s]\n', ...
        offsets_deg(j), rad2deg(x_s(3)), ...
        string(converged(j)).replace("true","CATCH").replace("false","DIVERGE"));
end

idx_fail = find(~converged, 1, 'first');
if isempty(idx_fail)
    fprintf('\nAll tested offsets caught — region of attraction > %d deg\n', offsets_deg(end));
elseif idx_fail == 1
    fprintf('\nNothing caught — region of attraction < %d deg\n', offsets_deg(1));
else
    fprintf('\nRegion of attraction edge: between %d and %d deg initial th2 offset\n', ...
        offsets_deg(idx_fail-1), offsets_deg(idx_fail));
end

%% ======================================================================
%% KNOWN DEFECTS IN MPC PATH (for awareness — not fixed here)
%% ======================================================================
%{
  1. run_mpc.m:15 passes a 2-element ref into compute_mpc, which subs into
     a 4-state symbolic vector — dimension mismatch / wrong operating point.
     run_controller.m (controller_type 3) passes the correct 4-vector.

  2. The R argument to compute_mpc is accepted but never used inside the
     function — per-reference R tuning has no effect.

  3. Output row 2 differs between production and test:
       compute_mpc.m: [1 0 1 0] = psi (th1 + th2)
       test.m:        [0 0 1 0] = raw th2
%}
