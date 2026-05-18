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
fprintf('Eigenvalues:\n');
for i = 1:length(ev)
    if imag(ev(i)) >= 0
        fprintf('  λ%d = %.4f + %.4fi', i, real(ev(i)), imag(ev(i)));
        if imag(ev(i)) ~= 0
            fprintf('   → fn = %.3f Hz', abs(imag(ev(i)))/(2*pi));
        end
        fprintf('\n');
    end
end

%% Natural frequencies from imaginary parts
fn = abs(imag(ev(imag(ev)>0))) / (2*pi);
fprintf('\nNatural frequencies: %.3f Hz and %.3f Hz\n', fn(1), fn(2));


fprintf('Natural frequency 1: %.2f Hz\n', fn(1));
fprintf('Natural frequency 2: %.2f Hz\n', fn(2));

%% Multisine design
h    = 0.01;          % sample period
Tsim = 30;            % experiment duration [s]
fs   = 1/h;           % 100 Hz sample rate
t    = (0:h:Tsim)';
N    = length(t);

% Frequency resolution
df = 1/Tsim;          % ~0.033 Hz per bin

% Coverage: from below fn(1) to above fn(2)
% Rule: cover 0.5*fn1 to 3*fn2
f_min = max(df, 0.5*fn(1));
f_max = min(3*fn(2), fs/5);     % stay well below Nyquist

% Select frequencies — every other bin to avoid harmonics
freq_indices = round(f_min/df) : 2 : round(f_max/df);
freqs = freq_indices * df;
fprintf('Exciting %d frequencies: %.2f to %.2f Hz\n', ...
        length(freqs), freqs(1), freqs(end));

% Schroeder phases — minimise peak-to-RMS ratio (best for hardware)
n_f    = length(freqs);
k      = 1:n_f;
phases = -pi * k.*(k-1) / n_f;   % Schroeder formula

% Build signal
u_raw = zeros(N, 1);
for i = 1:n_f
    u_raw = u_raw + sin(2*pi*freqs(i)*t + phases(i));
end

% Scale to safe amplitude — START VERY SMALL given 20x motor strength
A = 0.2;                          % normalised units — increase carefully
u_ms = A * u_raw / max(abs(u_raw));

fprintf('Signal RMS:  %.4f\n', rms(u_ms));
fprintf('Signal peak: %.4f\n', max(abs(u_ms)));
fprintf('Crest factor: %.2f  (lower is better, <3 is good)\n', ...
        max(abs(u_ms)) / rms(u_ms));

%% Check frequency content
U     = fft(u_ms) / N;
f_ax  = (0:N-1) * fs / N;

figure;
subplot(3,1,1);
plot(t, u_ms);
ylabel('u [-]'); xlabel('Time [s]');
title(sprintf('Multisine — %d frequencies, %.2f–%.2f Hz, A=%.3f', ...
              n_f, freqs(1), freqs(end), A));

subplot(3,1,2);
stem(f_ax(1:N/2), 2*abs(U(1:N/2)), 'filled', 'MarkerSize', 2);
xlim([0 fs/4]); xlabel('Frequency [Hz]'); ylabel('Amplitude');
xline(fn(1), 'r--', sprintf('f_{n1}=%.1fHz', fn(1)));
xline(fn(2), 'g--', sprintf('f_{n2}=%.1fHz', fn(2)));
title('Frequency spectrum — should cover both natural frequencies');

subplot(3,1,3);
plot(t, cumsum(u_ms)*h);   % approximate displacement
ylabel('Integrated u'); xlabel('Time [s]');
title('Integrated signal — check for drift');