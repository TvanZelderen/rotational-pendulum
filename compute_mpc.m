function [mpc_obj, Ad, Bd, Cd] = compute_mpc(x0, R, p, reference_indicator)

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
    

    if reference_indicator ==3
        Ts         = 0.01;  
        sys_c      = ss(A, B, C, D);
        sys_d      = c2d(sys_c, Ts, 'zoh');
        [Ad,Bd,Cd,Dd] = ssdata(sys_d);
    

        % Horizons
        Np = 30;    % prediction horizon — longer = better but slower
        Nc = 3;   % control horizon

        % Create MPC object
        mpc_obj = mpc(sys_d, Ts, Np, Nc);
    
        % Input constraints
        mpc_obj.MV.Min = -1;
        mpc_obj.MV.Max =  1;

        % Weights 
        % ManipulatedVariables: penalty on u magnitude
        % Higher = more conservative motor use
        mpc_obj.Weights.ManipulatedVariables     = 3;

        % ManipulatedVariablesRate: penalty on Δu (change in u)
        % Higher = smoother motor commands
        mpc_obj.Weights.ManipulatedVariablesRate = 0.01; 

        % OutputVariables: penalty on [th1 error, phi error]
        % Higher = track reference more aggressively
        % phi (pendulum angle) needs much higher weight than th1 (arm angle)

        mpc_obj.Weights.OutputVariables = [30, 150];    
   else   

        % Discretise
        Ts         = 0.001; 
        sys_c      = ss(A, B, C, D);
        sys_d      = c2d(sys_c, Ts, 'zoh');
        [Ad,Bd,Cd,Dd] = ssdata(sys_d);
    

        % Horizons
        Np = 500;    % prediction horizon — longer = better but slower
        Nc = 30;    % control horizon

        % Create MPC object
        mpc_obj = mpc(sys_d, Ts, Np, Nc);

        % Input constraints
        mpc_obj.MV.Min = -1;
        mpc_obj.MV.Max =  1;

        % Weights 
        % ManipulatedVariables: penalty on u magnitude
        % Higher = more conservative motor use
        mpc_obj.Weights.ManipulatedVariables     = 10; 

        % ManipulatedVariablesRate: penalty on Δu (change in u)
        % Higher = smoother motor commands
        mpc_obj.Weights.ManipulatedVariablesRate = 1; 

        % OutputVariables: penalty on [th1 error, phi error]
        % Higher = track reference more aggressively
        % phi (pendulum angle) needs much higher weight than th1 (arm angle)

        mpc_obj.Weights.OutputVariables = [100, 1000];    
    end   

    %% Nominal operating point
    mpc_obj.Model.Nominal.X = zeros(4,1);
    mpc_obj.Model.Nominal.Y = zeros(2,1);
    mpc_obj.Model.Nominal.U = 0;


end