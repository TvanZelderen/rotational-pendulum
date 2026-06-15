%% plot_lqr  —  load and plot a saved LQR/LQI run from data/
%
% Usage:
%   plot_lqr          → file picker dialog
%   plot_lqr('data/20260615_092416_lqr03dmHz.mat')  → load directly
%
% Column map (simout.Data):
%   1  th1    [deg]
%   2  dth1   [deg/s]
%   3  th2    [deg]
%   4  dth2   [deg/s]
%   5  psi    [-]  (control input u, post-saturation)
%   6  input  [rad] (e_input)
%   7  z_int  [rad·s]  (LQI integrator state — optional, when wired)

function plot_lqr(filepath)

data_dir = fullfile(fileparts(mfilename('fullpath')), 'data');

if nargin < 1 || isempty(filepath)
    [fname, fdir] = uigetfile('*.mat', 'Select LQR run', data_dir);
    if isequal(fname, 0), disp('Cancelled.'); return; end
    filepath = fullfile(fdir, fname);
end

s = load(filepath, 'simout');
t = s.simout.Time;
y = s.simout.Data;
ncols = size(y, 2);

th1   = y(:,1);
dth1  = y(:,2);
th2   = y(:,3);
dth2  = y(:,4);
u     = y(:,5);
e_in  = y(:,6);

has_lqi = ncols >= 7;
if has_lqi
    z_int = y(:,7);
end

[~, fname_only] = fileparts(filepath);

nrows = 3 + has_lqi;
figure('Name', fname_only); clf;

subplot(nrows,2,1);
plot(t, th1); yline(0,'k:');
ylabel('\theta_1 [deg]'); grid on; title('\theta_1');

subplot(nrows,2,2);
plot(t, th2); yline(0,'k:');
ylabel('\theta_2 [deg]'); grid on; title('\theta_2');

subplot(nrows,2,3);
plot(t, dth1);
ylabel('d\theta_1 [deg/s]'); grid on; title('d\theta_1');

subplot(nrows,2,4);
plot(t, dth2);
ylabel('d\theta_2 [deg/s]'); grid on; title('d\theta_2');

subplot(nrows,2,5);
plot(t, u); yline(1,'r--'); yline(-1,'r--');
ylabel('u [-]'); grid on; title('Input (post-sat)');

subplot(nrows,2,6);
plot(t, e_in);
ylabel('e_{input} [rad]'); grid on; title('Input error');

if has_lqi
    subplot(nrows,2,7);
    plot(t, z_int);
    ylabel('z_{int} [rad\cdots]'); xlabel('Time [s]'); grid on;
    title('LQI integrator state');
    subplot(nrows,2,8); axis off;   % blank — keep grid symmetric
end

sgtitle(strrep(fname_only, '_', '\_'), 'Interpreter','tex');
xlabel('Time [s]');

end
