fs = 1000;                % Sampling frequency (Hz)
t = 0:1/fs:1;             % Time vector (1 second)
x = sin(2*pi*50*t) + 0.5*randn(size(t));  % 50 Hz signal + noise

% Design 4th order low-pass Butterworth filter with 100 Hz cutoff
[b, a] = butter(4, 100/(fs/2));

% Apply zero-phase filtering
y = filtfilt(b, a, x);

% Plot original and filtered signals
plot(t, x, 'r--', t, y, 'b')
legend('Noisy signal', 'Filtered signal')
xlabel('Time (s)')
ylabel('Amplitude')
title('Low-pass Butterworth Filtering')
