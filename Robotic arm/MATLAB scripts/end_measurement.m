function end_measurement(src)
    configureCallback(src, "off");
    %plot(src.UserData.Data(2:end));
    src.UserData.Data(1,:) = [];  % deleting first vector of data only containing NaN
    plot_mpu(src.UserData.Data, src.UserData.Time)
    fprintf('data stream ended: %s\n', string(datetime('now', 'Format', 'HH-mm-ss')))
    if exist ("serialObj", "var")
       delete(serialObj)
    end
end