function end_measurement(src)
    configureCallback(src, "off");

    % Remove first row of NaNs
    src.UserData.Data(1,:) = [];

    % Just for reference 
    colNames = {
        'lin_acc_x', 'lin_acc_y', 'lin_acc_z', ...
        'ax', 'ay', 'az', ...
        'Yaw', 'Pitch', 'Roll', ...
        'roll_acc_deg', 'pitch_acc_deg'
    };

    % Plot raw data with timestamps
    plot_mpu(src.UserData.Data, src.UserData.Time);

    fprintf('data stream ended: %s\n', string(datetime('now', 'Format', 'HH-mm-ss')));

    if exist("serialObj", "var")
        delete(serialObj)
    end
end
