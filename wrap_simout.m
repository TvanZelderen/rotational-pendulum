function [th1, dth1, th2, dth2, psi] = wrap_simout(simout)
% Extract and wrap angle columns from simout to (-180, 180].
% Velocities (columns 2, 4) are returned unchanged.
    y    = simout.Data;
    th1  = mod(y(:,1) + 180, 360) - 180;
    dth1 = y(:,2);
    th2  = mod(y(:,3) + 180, 360) - 180;
    dth2 = y(:,4);
    psi  = mod(y(:,5) + 180, 360) - 180;
end
