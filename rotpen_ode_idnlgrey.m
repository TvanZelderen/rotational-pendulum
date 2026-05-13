function [dx, y] = rotpen_ode_idnlgrey(t, x, u, km, kb, c1, c2, J1, J2, l1, l2, lc1, m1, m2, g, varargin)
% Thin wrapper around rotpen_ode for use with idnlgrey / nlgreyest.
%
% idnlgrey calls this function with 12 separate scalar parameters in the
% order listed below.  Do NOT reorder — this order is the contract between
% the idnlgrey model object and this file.
%
% Parameter order (matches idnlgrey Parameters(1..12)):
%   1  km   — peak motor torque         [N·m per normalised unit]
%   2  kb   — back-EMF damping          [N·m·s/rad]
%   3  c1   — joint 1 viscous damping   [N·m·s/rad]
%   4  c2   — joint 2 viscous damping   [N·m·s/rad]
%   5  J1   — arm 1 inertia, pivot      [kg·m²]
%   6  J2   — arm 2 inertia, joint      [kg·m²]
%   7  l1   — arm 1 length              [m]
%   8  l2   — arm 2 length              [m]
%   9  lc1  — arm 1 CoM from pivot      [m]
%   10 m1   — arm 1 mass                [kg]
%   11 m2   — tip mass                  [kg]
%   12 g    — gravitational acceleration [m/s²]
%
% State:  x = [th1; dth1; th2; dth2]  (th2 relative, radians)
% Input:  u  normalised, u in [-1, +1]
% Output: y = x  (all 4 states observable)

    p.km  = km;
    p.kb  = kb;
    p.c1  = c1;
    p.c2  = c2;
    p.J1  = J1;
    p.J2  = J2;
    p.l1  = l1;
    p.l2  = l2;
    p.lc1 = lc1;
    p.m1  = m1;
    p.m2  = m2;
    p.g   = g;

    dx = rotpen_ode(t, x, u, p);
    y = x([1;3])
end
