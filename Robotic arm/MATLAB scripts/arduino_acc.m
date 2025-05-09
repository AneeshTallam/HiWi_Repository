%delete(serialportfind);

% Clear serialObj first, this needs also be done, if code on Arduino 
% needs to get updated
if exist ("serialObj", "var")
    delete(serialObj)
end
port = serialportlist("available");

serialObj = serialport("COM4", 115200);  %19200);
configureTerminator(serialObj,"CR/LF");
flush(serialObj);
serialObj.UserData = struct("Data", [NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN],"Time", ['00-00_00'], "Count",1);

maxDataPoints = 2500;
configureCallback(serialObj, "terminator", @(src,event) catcher_acc(src,event,maxDataPoints))
