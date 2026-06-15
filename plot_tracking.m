%% plot_tracking  —  load and plot a saved LQR reference-tracking run from data/
%
% Usage:
%   plot_tracking                              % file picker, ref from file
%   plot_tracking(filepath)                    % load file, ref from file
%   plot_tracking(filepath, ref_ind, DEMO)     % override / reconstruct ref
%
%   ref_ind : 1 = down-down | 2 = down-up | 3 = up-up
%   DEMO    : 0 = hold | 1 = step | 2 = sine | 3 = waypoints
%             4 = slow sine (up-up) | 5 = slow waypoints (up-up)
%
% If the .mat file contains ref_x (saved by recent run_controller runs),
% that is used directly. Otherwise ref_ind and DEMO are required to
% reconstruct the reference — pass them as arguments or enter when prompted.
%
% Column map (simout.Data):
%   1  th1  [deg]   3  th2  [deg]   5  psi [deg]   6  u [-]

function plot_tracking(filepath, reference_indicator, DEMO)

data_dir = fullfile(fileparts(mfilename('fullpath')), 'data');

if nargin < 1 || isempty(filepath)
    [fname, fdir] = uigetfile('*.mat', 'Select tracking run', data_dir);
    if isequal(fname, 0), disp('Cancelled.'); return; end
    filepath = fullfile(fdir, fname);
end

vars = whos('-file', filepath);
var_names = {vars.name};
has_ref = ismember('ref_x', var_names);

if has_ref
    s = load(filepath, 'simout', 'ref_x', 'ref', 'ttl');
    ref_eq  = s.ref;
    t_vec   = s.ref_x.Time;
    ref_dev = s.ref_x.Data;
    ttl     = s.ttl;
else
    s = load(filepath, 'simout');

    if nargin < 2 || isempty(reference_indicator)
        reference_indicator = input('ref_ind (1=down-down, 2=down-up, 3=up-up): ');
    end
    if nargin < 3 || isempty(DEMO)
        DEMO = input('DEMO (0=hold, 1=step, 2=sine, 3=waypoints, 4=slow-sine, 5=slow-wp): ');
    end

    % Equilibrium per reference_indicator
    switch reference_indicator
        case 1; ref_eq = [0; 0; 0; 0];
        case 2; ref_eq = [0; 0; pi; 0];
        case 3; ref_eq = [-pi; 0; 0; 0];
    end

    % Trajectory per DEMO — mirrors run_controller.m switch
    h = 0.001;
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
            ttl      = 'Sine sweep: ±30°, 0.15 Hz';
        case 5
            run_time = 20;
            ref_fn   = @(t) deg2rad(30)*(t >= 5 & t < 10) + deg2rad(-30)*(t >= 10 & t < 15);
            ttl      = 'Waypoints: 0° → +30° → −30° → 0°';
    end

    t_vec       = (0 : h : run_time)';
    th1_ref_dev = arrayfun(ref_fn, t_vec);
    th2_ref_dev = -th1_ref_dev;
    ref_dev     = [th1_ref_dev, zeros(size(t_vec)), th2_ref_dev, zeros(size(t_vec))];
end

t   = s.simout.Time;
y   = s.simout.Data;
th1 = y(:,1);
th2 = y(:,3);
u   = y(:,6);

th1_ref_deg = rad2deg(ref_eq(1)) + rad2deg(ref_dev(:,1));
th2_ref_deg = rad2deg(ref_eq(3)) + rad2deg(ref_dev(:,3));

[~, fname_only] = fileparts(filepath);

figure('Name', fname_only); clf;

subplot(3,1,1);
plot(t, th1, 'b', t_vec, th1_ref_deg, 'k--', 'LineWidth', 1.2);
ylabel('\theta_1 [deg]'); legend('actual', 'reference'); grid on;
title(['Reference tracking — ' ttl]);

subplot(3,1,2);
plot(t, th2, 'r', t_vec, th2_ref_deg, 'k--', 'LineWidth', 1.2);
yline(rad2deg(ref_eq(3)), 'k:');
ylabel('\theta_2 [deg]'); legend('actual', 'reference'); grid on;

subplot(3,1,3);
plot(t, u, 'k'); yline(1, 'r--'); yline(-1, 'r--');
ylabel('u [-]'); xlabel('Time [s]'); grid on;

sgtitle(strrep(fname_only, '_', '\_'), 'Interpreter', 'tex');

end
