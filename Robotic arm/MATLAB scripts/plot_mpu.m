    function plot_mpu(data, time)

    %save(sprintf('ypr_arduino_%s.mat', datetime("now", 'InputFormat', 'yyyy-MM-dd')), 'data')
    save(sprintf('ypr_arduino_%s.mat', string(datetime('now', 'Format', 'HH-mm-SS'))), 'data')
    save(sprintf('time_arduino_%s.mat', string(datetime('now', 'Format', 'HH-mm-SS'))), 'time')
    figure
    subplot(5,1,1)
    plot(data(:,7))
    subtitle("yaw")
    subplot(5,1,2)
    plot(data(:,8))
    subtitle("pitch")
    subplot(5,1,3)
    plot(data(:,9))
    subtitle("roll")
    ax = data(:,4);
    ay = data(:,5);
    az = data(:,6);
    pitch_accel = -atan2(ax, sqrt(ay.^2 + az.^2)) * (180 / pi);
    roll_accel = atan2(ay, sqrt(ax.^2 + az.^2)) * (180 / pi);
    fs = 100; %Sampling rate
    fc = 10; %cut off frequency
    [b, a] = butter(4, fc / (fs / 2));
    y = filtfilt(b,a,roll_accel);
    subplot(5,1,4)
    plot(pitch_accel)
    subtitle('Roll accel')
    subplot(5,1,5)
    plot(y)
    subtitle('Filtered roll accel')
end