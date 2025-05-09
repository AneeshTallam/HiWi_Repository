% Create GUI
h.fig = figure();
h.but1 = uicontrol(h.fig,'Style','PushButton','Units','Normalize','Position',[.1,.8,.2,.1],'String','Go');
h.but2 = uicontrol(h.fig,'Style','PushButton','Units','Normalize','Position',[.1,.6,.2,.1],'String','Stop');
h.but2Val = false;  % Default stop-button-off
guidata(h.fig, h)
h.but1.Callback = {@button1CallbackFcn,h};
h.but2.Callback = {@button2CallbackFcn,h};
function button1CallbackFcn(hObj,event,handles)
    
        %delete(serialportfind);
    
    % Clear serialObj first, this needs also be done, if code on Arduino 
    % needs to get updated
    if exist ("serialObj", "var")
        delete(serialObj)
    end
    port = serialportlist("available");
    
    serialObj = serialport("COM5", 115200);  %19200);
    configureTerminator(serialObj,"CR/LF");
    flush(serialObj);
    serialObj.UserData = struct("Data",[NaN, NaN, NaN],"Time", [], "Count",1);
    
    maxDataPoints = 5000; 
    rec = true;
    configureCallback(serialObj, "terminator", @(src,event) catcher(src,event,maxDataPoints))
    handles = guidata(hObj.Parent);

    if handles.but2Val
        handles.but2Val = false;  % reset stop-button to off
        guidata(hObj.Parent,handles)
        end_measurement(src)
    end
        
end

function button2CallbackFcn(hObj,event,handles)
% Responds to "stop" button.  Toggles the but2Val to TRUE
handles.but2Val = true; 
guidata(hObj.Parent, handles)
end