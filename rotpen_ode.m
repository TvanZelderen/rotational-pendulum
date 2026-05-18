function dxdt = rotpen_ode(~, x, u, p)
    % Equations of motion for the planar rotational pendulum.
    % Both arms swing in the same vertical plane; the motor axis is horizontal.
    %
    % State vector:
    %   x(1) = theta1   [rad]   arm 1 angle from vertical downward  (theta1=0: hanging)
    %   x(2) = dtheta1  [rad/s]
    %   x(3) = theta2   [rad]   arm 2 angle relative to arm 1 extension (theta2=0: aligned)
    %   x(4) = dtheta2  [rad/s]
    %
    % Input:
    %   u  [-]   normalised motor input, u in [-1, +1]  (NOT raw volts)
    %   p        parameter struct from pendulum_params.m
    
    %% Unpack state
    th1  = x(1);
    dth1 = x(2);
    th2  = x(3);
    dth2 = x(4);
    
    %% Motor torque on joint 1
    % Assumption A2 (stiff arm): reaction forces from arm 2 back onto arm 1 are
    % negligible, so tau drives arm 1 independently.  The full coupling term
    % still appears in M and rhs below (more accurate); dropping it is a
    % simplification to validate later against hardware data.
    tau = -p.km * u;   % [N·m]   velocity-dependent braking lumped into kbc1 below
    
    % -----------------------------------------------------------------------
    % TODO — Derive the equations of motion using the Euler-Lagrange method.
    %
    % The system has two degrees of freedom: theta1 and theta2.
    % You will need to:
    %
    %  1. Write the Cartesian position of each body's centre of mass.
    %
    %     CoM of arm 1:
    %       x_c1 = p.lc1 * sin(th1)
    %       y_c1 = -p.lc1 * cos(th1)
    %
    %     Joint position (end of arm 1):
    %       x_j  = p.l1 * sin(th1)
    %       y_j  = -p.l1 * cos(th1)
    %
    %     CoM of ball (tip of arm 2):
    %       x_c2 = x_j + p.l2 * sin(th1 + th2)       <-- why (th1 + th2)?
    %       y_c2 = y_j - p.l2 * cos(th1 + th2)
    %
    %  2. Differentiate to get velocities, then write the kinetic energy:
    %       T = (1/2)*p.J1*dth1^2 + (1/2)*p.m2*(v_c2x^2 + v_c2y^2)
    %     (arm 1 is a rigid rod so its KE is already captured by J1 = 1/3 m1 l1^2)
    %
    %  3. Write the potential energy:
    %       V = -p.m1*p.g*p.lc1*cos(th1) - p.m2*p.g*( ... )
    %
    %  4. Lagrangian:  L = T - V
    %     Apply Euler-Lagrange for each generalised coordinate:
    %       d/dt(dL/d(dth1)) - dL/d(th1) = tau - p.kbc1*dth1
    %       d/dt(dL/d(dth2)) - dL/d(th2) =     - p.c2*dth2
    %
    %  5. You will arrive at the form:
    %       M(th2) * [ddth1; ddth2] = rhs
    %     where M is a 2×2 inertia matrix and rhs collects everything else.
    %     Solve for accelerations with:
    %       ddq = M \ rhs;
    %
    % Tip: expand v_c2x and v_c2y before squaring — you'll get a cos(th2) term
    %      in the off-diagonal of M. That coupling is the heart of the problem.
    % -----------------------------------------------------------------------
    
    %% Inertia matrix  M(th2)  — 2×2, function of th2 only
    M = [p.J1+p.m2*(p.l1^2+p.l2^2+2*p.l1*p.l2*cos(th2)), p.m2*(p.l2^2+p.l1*p.l2*cos(th2)); p.m2*(p.l2^2+p.l1*p.l2*cos(th2)), p.m2*p.l2^2];
    
    %% Right-hand side  (Coriolis + gravity + damping + input)
    h = p.m2*p.l1*p.l2*sin(th2);
    v_c = 0.1;   % Coulomb smoothing velocity [rad/s]; tanh(dth1/v_c) ≈ sign(dth1) for |dth1| >> v_c
    rhs = [tau - p.kbc1*dth1 - p.tauc_kinetic*tanh(dth1/v_c) - (-h*(2*dth1*dth2+dth2^2)) - ((p.m1*p.lc1+p.m2*p.l1)*p.g*sin(th1) + p.m2*p.g*p.l2*sin(th1+th2)); ...
           -p.c2*dth2 - (h*dth1^2) - (p.m2*p.g*p.l2*sin(th1+th2))];
    
    %% Solve for angular accelerations
    ddq = M \ rhs;
    
    %% Assemble state derivative
    dxdt      = zeros(4, 1);
    dxdt(1)   = dth1;
    dxdt(2)   = ddq(1);
    dxdt(3)   = dth2;
    dxdt(4)   = ddq(2);
end
