function catcher_acc(src, ~, maxDataPoints)
    persistent flag_first_real_value_sent
    persistent pitch_buffer roll_buffer

    if isempty(flag_first_real_value_sent)
        flag_first_real_value_sent = false;
    end
    if isempty(pitch_buffer)
        pitch_buffer = [];
        roll_buffer = [];
    end

    data_str = readline(src);
    if data_str ~= ""
        if ~(data_str(end) >= '0' && data_str(end) <= '9')
            return;  % Skip malformed line
        end

        data_str_split = strsplit(data_str, ',');
        data = cellfun(@str2double, data_str_split);

        if length(data) < 6
            return;  % Skip if not enough values
        end

        ax = data(4);
        ay = data(5);
        az = data(6);

        pitch_accel = -atan2(ax, sqrt(ay^2 + az^2)) * (180 / pi);
        roll_accel  =  atan2(ay, sqrt(ax^2 + az^2)) * (180 / pi);

        pitch_buffer(end+1) = pitch_accel;
        roll_buffer(end+1)  = roll_accel;

        % Optional: limit buffer length
        maxBufferLength = 500;
        if length(pitch_buffer) > maxBufferLength
            pitch_buffer = pitch_buffer(end-maxBufferLength+1:end);
            roll_buffer  = roll_buffer(end-maxBufferLength+1:end);
        end

        % Only filter if enough data (min 13 samples for filtfilt)
        if length(pitch_buffer) >= 25
            fs = 100; fc = 10;
            [b, a] = butter(4, fc / (fs / 2));
            pitch_filtered = filtfilt(b, a, pitch_buffer);
            roll_filtered  = filtfilt(b, a, roll_buffer);

            pitch_val = pitch_filtered(end);
            roll_val  = roll_filtered(end);
        else
            pitch_val = pitch_accel;
            roll_val  = roll_accel;
        end

        data = [data, roll_val, pitch_val];

        if ~flag_first_real_value_sent && ~anynan(data)
            flag_first_real_value_sent = true;
            fprintf('data stream started: %s\n', string(datetime('now', 'Format', 'HH-mm-ss')))
        end

        src.UserData.Data(end+1,:) = data;
        src.UserData.Time(end+1,:) = string(datetime('now', 'Format', 'HH-mm-ss'));
        src.UserData.Count = src.UserData.Count + 1;
    end

    if src.UserData.Count > maxDataPoints
        end_measurement(src)
    end
end
