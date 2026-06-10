function [mpc_obj, Ad, Bd, Cd] = compute_mpc(x0, R, p)

    %% Linearise
    syms x [4 1] real
    syms u real
    dxdt_sym = rotpen_ode(0, x, u, p);
    A_sym    = jacobian(dxdt_sym, x);
    B_sym    = jacobian(dxdt_sym, u);
    A = double(subs(subs(A_sym, x, x0), u, 0));
    B = double(subs(subs(B_sym, x, x0), u, 0));

    %% Use only MEASURED outputs (angles only — matches your hardware)
    % MPC should only track what you can actually measure
    C = [1 0 0 0;  % th1
         1 0 1 0];  % th2
    D = zeros(2,1);

    %% Discretise
    Ts         = 0.001;
    sys_c      = ss(A, B, C, D);
    sys_d      = c2d(sys_c, Ts, 'zoh');
    [Ad,Bd,Cd,Dd] = ssdata(sys_d);
    

    %% Horizons
    Np = 500;    % prediction horizon — longer = better but slower
    Nc = 30;    % control horizon

    %% Create MPC object
    mpc_obj = mpc(sys_d, Ts, Np, Nc);

    %% Input constraints
    mpc_obj.MV.Min = -1;
    mpc_obj.MV.Max =  1;

    %% Weights — this is the key tuning
    % ManipulatedVariables: penalty on u magnitude
    % Higher = more conservative motor use
    mpc_obj.Weights.ManipulatedVariables     = 1;

    % ManipulatedVariablesRate: penalty on Δu (change in u)
    % Higher = smoother motor commands
    mpc_obj.Weights.ManipulatedVariablesRate = 0.01;

    % OutputVariables: penalty on [th1 error, th2 error]
    % Higher = track reference more aggressively
    % th2 (pendulum angle) needs much higher weight than th1 (arm angle)

    mpc_obj.Weights.OutputVariables = [100, 1000];    
   

    %% Nominal operating point
    mpc_obj.Model.Nominal.X = x0;
    mpc_obj.Model.Nominal.Y = C * x0;
    mpc_obj.Model.Nominal.U = 0;

    %% Scale factors — help MPC numerics
    % Set to expected range of each variable
%     mpc_obj.OV(1).ScaleFactor = deg2rad(30);      % th1 range ~ pi rad
%     mpc_obj.OV(2).ScaleFactor = deg2rad(30);      % th2 range ~ pi rad
%     mpc_obj.MV.ScaleFactor    = 1;       % u range ~ 


end