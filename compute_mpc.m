function[mpc_obj,Ad,Bd,Cd] = compute_mpc(x0,R,p)
syms x [4 1] real
syms u real

dxdt_sym = rotpen_ode(0, x, u, p);

A_sym = jacobian(dxdt_sym, x);
B_sym = jacobian(dxdt_sym, u);

A = double(subs(subs(A_sym, x, x0), u, 0));
B = double(subs(subs(B_sym, x, x0), u, 0));
C = [1 0 0 0; 
     0 0 1 0]; 
D = zeros(2,1);

%Discretize the state space system
Ts = 0.001;
sys_continuous = ss(A, B, C, 0);
sys_discrete = c2d(sys_continuous, Ts, 'zoh');
[Ad, Bd, Cd, Dd] = ssdata(sys_discrete);

%Prediction and Control horizon
Np = 20; % predicts Np time-steps ahead 
Nc = 3; % Nc time-steps controller sets its input

%MPC object
mpc_obj = mpc(sys_discrete, Ts, Np, Nc);
mpc_obj.MV.Min = -1; % Max (just like the saturation block but it needs this to predict)
mpc_obj.MV.Max = 1;  % Max 

mpc_obj.Weights.ManipulatedVariables = R;         % 'R' penalty (motor effort)
mpc_obj.Weights.ManipulatedVariablesRate = 0.1;   % Penalty on violently changing voltage
mpc_obj.Weights.OutputVariables = [10, 0, 10, 0]; % 'Q' penalty (positions)