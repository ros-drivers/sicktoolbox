function lms_stream(sick_dev_path,sick_baud)
%==========================================================================
%==========================================================================
%
%  The Sick LIDAR Matlab/C++ Toolbox
%
%  File: lms_stre.m
%  Auth: Jason C. Derenick and Thomas H. Miller
%  Cont: derenick(at)lehigh(dot)edu
%  Date: 11 January 2008
%
%  In:   sick_dev_path  - Sick device path
%        sick_baud_rate - Desired baud rate of comm session
%  Out:  None
%
%  Desc:
%        This function grabs measurements from the Sick and displays them.
%        To stop the Sick and the program, simply close the figure window.
%
%        Usage:   lms_stream(SICK_DEV_PATH,SICK_BAUD)
%        Example: lms_stream('/dev/ttyUSB0',500000)
%
%==========================================================================
global keep_running;

% Check args are given
if nargin ~= 2
    error('Invalid num of input args! Type "help lms_stream" for example usage.'); 
end

% Close all open figures
close all;

% Initialize plots
ran_plot = [];
ref_plot = [];

% Initialize the device
init_res = sicklms('init',sick_dev_path,sick_baud);
if isempty(init_res), error('sicklms init failed!'); end

disp('        *** CLOSE FIGURE TO QUIT ***');
disp(' ');

% Main process control loop
keep_running = 1;
while(keep_running)
    
    % Start data stream and grab measurements
    data = sicklms('grab');
    if isempty(data), error('sicklms grab failed!'); end
    
    % If range data is available then create/update range plot
    if ~isempty(data.range)
        
        % If necessary create the figure
        if isempty(ran_plot)
           ran_plot = setupAxes(-data.fov/2, ...
                                 data.fov/2, ...
                                 data.res, ...
                                 30, ...
                                 'b', ...
                                 '\theta (deg)', ...
                                 'Distance (m)', ...
                                 'Sick LMS Range');
        end
        
        % Normalize units by setting to meters
        % NOTE: units_mm is a boolean indicating whether units are in (mm)
        data.range = toMeters(data.range,init_res.units_mm);
        
        % Set the range plot data
        setPlotData(data.range,ran_plot);
        
    end

    % If reflectivity data is available create/update reflect plot
    if ~isempty(data.reflect)
        
        if isempty(ref_plot)
            ref_plot = setupAxes(-data.fov/2, ...
                                  data.fov/2, ...
                                  data.res, ...
                                  255, ...
                                  'r', ...
                                  '\theta (deg)', ...
                                  '\gamma', ...
                                  'Sick LMS Reflectivity');
        end
       
        % Set the range plot data
        setPlotData(data.reflect,ref_plot);
            
    end
    
    drawnow; % Redraw the window(s)
    
end

% Uninitialize the device
clear sicklms;
close all;

end % lms_stream

%==========================================================================
%==========================================================================
function lms_plot = setupAxes(x_lb,x_ub,x_res,y_ub, ...
                              color_str,x_label_str,y_label_str,title_str)
%==========================================================================
% Func: setupRangeAxes()
% Desc: Creates an axes object for displaying range data
%==========================================================================
figure('DeleteFcn',@closeProgram,'Name','Sick LMS Data Stream');

% Setup the range subplot (after checking what type of plot to do)
lms_plot.h = subplot(1,1,1);

axis([x_lb x_ub 0 y_ub]);
title(title_str);
xlabel(x_label_str);
ylabel(y_label_str);
set(gca,'box','on');
grid on;

% Create range line object
x_axis = x_lb:x_res:x_ub;
y_axis = zeros(1,length(x_axis));
lms_plot.line = line('XData',x_axis,'YData',y_axis, ...
                     'Marker','.','LineStyle','none','Color',color_str);
lms_plot.color = color_str;

end % function setupAxes

%==========================================================================
%==========================================================================
function setPlotData(values,lms_plot)
%==========================================================================
% Func: setPlotData()
% Desc: Adjusts the line data to reflect the given measurement values
%==========================================================================

% Update the range line
set(lms_plot.line,'ydata',values,'color',lms_plot.color);

end % function setPlotData

%==========================================================================
%==========================================================================
function range = toMeters(range,units_mm)
%==========================================================================
% Func: toMeters()
% Desc: Converts given measurements to meters 
%==========================================================================

if units_mm % Units are (mm)
    range = range./1000;
else % Units are (cm)
    range = range./100;
end

end % toMeters

%==========================================================================
%==========================================================================
function closeProgram(src,var)
%==========================================================================
% Func: closeProgram()
% Desc: Indirectly stops the display and uninitializes the Sick
% Note: By setting the global keep_running to 0, this breaks out of the
%       while loop. This will execute when the figure window is closed. 
%==========================================================================
global keep_running;
keep_running = 0;

end % closeProgram
