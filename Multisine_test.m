%% Natural frequencies from known parameters
pendulum_params;   % loads struct p

%% Choose equilibrium point
% Stable equilibrium: both links hanging down
x_eq = [0; 0; 0; 0];   % [th1, dth1, th2, dth2]
u_eq = 0;               % no input at equilibrium

%% Numerical Jacobian via finite differences
n  = 4;        % number of states
nu = 1;        % number of inputs
eps = 1e-6;    % perturbation size

f0 = rotpen_ode(0, x_eq, u_eq, p);   % baseline

% A matrix — perturb each state for linearizartion 
A = zeros(n, n);
for i = 1:n
    x_pert    = x_eq;
    x_pert(i) = x_pert(i) + eps;
    f_pert    = rotpen_ode(0, x_pert, u_eq, p);
    A(:, i)   = (f_pert - f0) / eps;
end

% B matrix — perturb input
B = zeros(n, nu);
f_pert = rotpen_ode(0, x_eq, u_eq + eps, p);
B(:,1) = (f_pert - f0) / eps;

% C matrix — you only measure th1 and th2
C = [1 0 0 0;   % th1
     0 0 1 0];  % th2
D = zeros(2, 1);

fprintf('\n--- Linearised system at stable equilibrium ---\n');
disp('A ='); disp(A)
disp('B ='); disp(B)

%% Eigenvalues
ev = eig(A);

% Get all eigenvalues with nonzero imaginary part (positive only, avoid duplicates)
ev_complex = ev(imag(ev) > 1e-6);   % threshold avoids near-zero numerical noise
fn = sort(abs(imag(ev_complex)) / (2*pi));   % sort low to high

fprintf('\nEigenvalues (all 4):\n');
for i = 1:length(ev)
    fprintf('  λ%d = %+.4f %+.4fj\n', i, real(ev(i)), imag(ev(i)));
end

fprintf('\nNatural frequencies found: %d\n', length(fn));
for i = 1:length(fn)
    fprintf('  fn%d = %.3f Hz\n', i, fn(i));
end

% Safe access
if length(fn) >= 2
    fprintf('\nNatural frequencies: %.3f Hz and %.3f Hz\n', fn(1), fn(2));
elseif length(fn) == 1
    fprintf('\nOnly 1 oscillatory mode found: %.3f Hz\n', fn(1));
    fprintf('Other mode is overdamped (purely real eigenvalue)\n');
else
    fprintf('\nNo oscillatory modes — system is fully overdamped\n');
end

%% Multisine design — works with 1 or 2 natural frequencies
h    = 0.01;
Tsim = 30;
fs   = 1/h;
t    = (0:h:Tsim)';
N    = length(t);
df   = 1/Tsim;

% Use whatever natural frequencies were found
if length(fn) >= 2
    f_center_low  = fn(1);
    f_center_high = fn(2);
elseif length(fn) == 1
    f_center_low  = fn(1);
    f_center_high = fn(1);   % only one mode — design around it
    fprintf('Single mode system — designing multisine around %.3f Hz\n', fn(1));
else
    f_center_low  = 1.0;     % fallback if fully overdamped
    f_center_high = 5.0;
    fprintf('No oscillatory modes found — using default frequency range\n');
end

% Coverage: 0.1*fn_low to 3*fn_high
f_min = max(df, 0.1 * f_center_low);
f_max = min(3   * f_center_high, fs/5);

freq_indices = round(f_min/df) : 2 : round(f_max/df);
freqs = freq_indices * df;
n_f   = length(freqs);

fprintf('Multisine: %d frequencies from %.3f to %.3f Hz\n', n_f, freqs(1), freqs(end));

% Schroeder phases
k      = 1:n_f;
phases = -pi * k.*(k-1) / n_f;

% Build signal
u_raw = zeros(N, 1);
for i = 1:n_f
    u_raw = u_raw + sin(2*pi*freqs(i)*t + phases(i));
end

A_amp = 0.02;   % start very small — your motor is 20x stronger than expected
u_ms  = A_amp * u_raw / max(abs(u_raw));

fprintf('RMS = %.4f  |  Peak = %.4f  |  Crest factor = %.2f\n', ...
        rms(u_ms), max(abs(u_ms)), max(abs(u_ms))/rms(u_ms));

%% Plot
U     = fft(u_ms) / N;
f_ax  = (0:N-1) * fs / N;

figure;
subplot(2,1,1);
plot(t, u_ms);
ylabel('u [-]'); xlabel('Time [s]');
title(sprintf('Multisine — %d freqs, %.2f–%.2f Hz, A=%.3f', n_f, freqs(1), freqs(end), A_amp));

subplot(2,1,2);
stem(f_ax(1:N/2), 2*abs(U(1:N/2)), 'filled', 'MarkerSize', 2);
xlim([0 min(3*f_center_high + 2, fs/4)]);
xlabel('Frequency [Hz]'); ylabel('Amplitude');
for i = 1:length(fn)
    xline(fn(i), 'r--', sprintf('f_n=%.2fHz', fn(i)));
end
title('Frequency spectrum');