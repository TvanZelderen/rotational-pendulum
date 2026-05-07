%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DSCS FPGA interface board: init and I/O conversions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Calibration formula (applied in Simulink):
%   calibrated = (raw + adinoffs) * adingain
%
% adinoffs: stored as -[raw_mean_at_equilibrium], so adding it centres the signal
% adingain: 360 / range_over_one_full_rotation  →  converts raw units to degrees
%
% Values derived 2025-05-01: zero-input steady-state with both links hanging down.

% Output (D/A)
daoutoffs = [0.00];
daoutgain = 1*[-6];

% Sensor calibration: theta1, theta2
adinoffs = -[-3.7842, -1.2037];      % negated raw means at equilibrium
adingain = [360/4.9036, 360/4.9390]; % raw-unit to degrees gains

% Remaining channels: switches and encoder (no calibration needed)
adinoffs = [adinoffs 0 0 0 0 0];
adingain = [adingain 1 1 1 1 1];
