function [dx, y] = rotpen_ode_idnlgrey(t, x, u, km, kbc1, c2, J1, J2, l1, l2, lc1, m1, m2, g, tauc_kinetic, varargin)
% Thin wrapper around rotpen_ode for use with idnlgrey / nlgreyest.
%
% idnlgrey calls this function with 12 separate scalar parameters in the
% order listed below.  Do NOT reorder — this order is the contract between
% the idnlgrey model object and this file.
%
% Parameter order (matches idnlgrey Parameters(1..12)):
%   1  km           — peak motor torque                          [N·m per normalised unit]
%   2  kbc1         — composite velocity damping: kb + c1        [N·m·s/rad]
%   3  c2           — joint 2 viscous damping                    [N·m·s/rad]
%   4  J1           — arm 1 inertia, pivot                       [kg·m²]
%   5  J2           — arm 2 inertia, joint                       [kg·m²]
%   6  l1           — arm 1 length                               [m]
%   7  l2           — arm 2 length                               [m]
%   8  lc1          — arm 1 CoM from pivot                       [m]
%   9  m1           — arm 1 mass                                 [kg]
%   10 m2           — tip mass                                   [kg]
%   11 g            — gravitational acceleration                  [m/s²]
%   12 tauc_kinetic — Coulomb kinetic friction, arm 1            [N·m]
%
% State:  x = [th1; dth1; th2; dth2]  (th2 relative, radians)
% Input:  u  normalised, u in [-1, +1]
% Output: y = [th1; th2]

    p.km           = km;
    p.kbc1         = kbc1;
    p.c2           = c2;
    p.J1           = J1;
    p.J2           = J2;
    p.l1           = l1;
    p.l2           = l2;
    p.lc1          = lc1;
    p.m1           = m1;
    p.m2           = m2;
    p.g            = g;
    p.tauc_kinetic = tauc_kinetic;

    dx = rotpen_ode(t, x, u, p);
    y  = x([1; 3]);
end
