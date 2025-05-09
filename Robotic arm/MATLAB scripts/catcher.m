function catcher(src, ~, maxDataPoints)
    persistent flag_first_real_value_sent
    if isempty(flag_first_real_value_sent)
        flag_first_real_value_sent = false;
    end
    % Read the ASCII data from the serialport object.
    data_str = readline(src);
    if data_str ~= ""
        if ~(data_str(end) >= '0' && data_str(end) <='9') %checks if last character is not a number
            disp(data_str)
        else
            data_str_split = strsplit(data_str, ',');
            data = cellfun(@str2double, data_str_split);
            
            if ~flag_first_real_value_sent && ~anynan(data) 
                flag_first_real_value_sent = true;
                fprintf('data stream started: %s\n', string(datetime('now', 'Format', 'HH-mm-ss')))
            end      
    
                   
    
            % Convert the string data to numeric type and save it in the UserData
            % property of the serialport object.
            src.UserData.Data(end+1,:) = data;
            src.UserData.Time(end+1,:) = string(datetime('now', 'Format', 'HH-mm-ss'));
        
            % Update the Count value of the serialport object.
            src.UserData.Count = src.UserData.Count + 1;
    
            % src.UserData.Time = datetime();
        end
    end
    %disp(src.UserData.Count)
    % If over maxDataPoints points have been collected from the Arduino, switch off the
    % callbacks and plot the data, starting from the second point. 
    if src.UserData.Count > maxDataPoints
        end_measurement(src)
    end
end