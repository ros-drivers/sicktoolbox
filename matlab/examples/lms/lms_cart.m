function lms_cart(sick_dev_path,sick_baud)
%==========================================================================
%==========================================================================
%
%  The Sick LIDAR Matlab/C++ Toolbox
%
%  File: lms_cart.m
%  Auth: Jason C. Derenick and Thomas H. Miller
%  Cont: derenick(at)lehigh(dot)edu
%  Date: 11 January 2008
%
%  In:   sick_dev_path  - Sick device path
%        sick_baud_rate - Desired baud rate of comm session
%  Out:  None
%
%  Desc:
%        This function uses the data streaming from the Sick to create a
%        "top-down" view of the surrounding environment.
%
%        To stop the Sick and the program, simply close the figure window.
%
%        Usage:   lms_cart(SICK_DEV_PATH,SICK_BAUD)
%        Example: lms_cart('/dev/ttyUSB0',500000)
%
%  NOTE: If the device is not an LMS Fast, it should be in a measurement
%        mode that allows streaming range information for this example to
%        work properly.
%
%==========================================================================
global keep_running

% Check args are given
if nargin ~= 2
    error('Invalid num of input args! Type "help lms_cart" for example usage.');
end

% Close all open figures
close all;

% Define the plot bounds (units)
lms_plot.x.ub =  10;
lms_plot.x.lb = -10;
lms_plot.y.ub =  10;
lms_plot.y.lb =   0;

% Initialize the Sick LMS 2xx
init_res = sicklms('init',sick_dev_path,sick_baud);
if isempty(init_res), error('sicklms init failed!'); end

% Initialize the plot figure
lms_plot = setupAxes(lms_plot,init_res.lms_fast);

disp('        *** CLOSE FIGURE TO QUIT ***');
disp(' ');

% Main process control loop
keep_running = 1;
while keep_running

    % Grab LMS data
    data = sicklms('grab');    
    if isempty(data), error('sicklms grab failed!'); end
    
    % Check that the device is returning range
    if isempty(data.range)
        error('Empty range data! Is the measuring mode set properly?'); 
    end

    % For filtering overflow (e.g. max range) values returned from LMS
    valid_indices = data.range < overflowValue(init_res.meas_mode);

    % Keep only valid (non-overflow) measurements
    data.range = data.range(valid_indices);    
    if ~isempty(data.reflect) 
        data.reflect = data.reflect(valid_indices);        
    end
    
    % Normalize units as meters (only for plotting purposes)
    % NOTE: units_mm is a boolean indicating whether units are mm
    data.range = toMeters(data.range,init_res.units_mm);
    
    % Generate angles corresponding to measurements
    theta = (data.fov/2:-data.res:-data.fov/2)'*pi/180;
    theta = theta(valid_indices);
    
    % Convert polar to Cartesian coordinates (in Sick LMS frame)    
    [x_pos y_pos] = pol2cart(theta,data.range);
    
    % Update the plot data    
    setPlotData(lms_plot.p_obj,y_pos,x_pos,data.reflect);            
    drawnow; % Redraw the plot
        
end

% Uninitialize the device
clear sicklms;

end % function ld_cart

%==========================================================================
%==========================================================================
function lms_plot = setupAxes(lms_plot,lms_fast)
%==========================================================================
% Func: setupAxes()
% Desc: Creates an axes object for displaying the data stream
%==========================================================================
figure('DeleteFcn',@closeProgram,'Name','Sick LMS Cartesian Plot');
lms_plot.h = subplot(1,1,1);
axis(lms_plot.h,[lms_plot.x.lb lms_plot.x.ub lms_plot.y.lb lms_plot.y.ub]);
xlabel('Displacement (m)');
ylabel('Displacement (m)');
title('Sick LMS Cartesian Plot')

% Use a single patch for plotting all values
edge_color = 'flat';
if lms_fast
    edge_color = 'interp';
end

% Create the patch object to represent scan pts
lms_plot.p_obj = patch('Parent',lms_plot.h, ...
                       'Marker','.', ...
                       'EdgeColor',edge_color, ...
                       'EdgeAlpha',0, ...
                       'LineStyle','none', ...
                       'FaceColor','none'); 

% Generate the sick graphic                  
patch([-0.08 -0.08 0.08 0.08], ...
      [    0  0.08 0.08    0], ...
      [0 0 1]);
  
set(gca,'box','on');
grid on;                        

end % function setupAxes

%==========================================================================
%==========================================================================
function overflow = overflowValue(meas_mode)
%==========================================================================
% Func: overflowValue()
% Desc:  This function maps the given measuring mode value to the correct
%        overflow value so that Sick LMS returns can be properly filtered
%==========================================================================
overflow = inf;

% NOTE: These values are from page 124 of LMS Telegram Listing
switch meas_mode
    case {0,1,2};
        overflow = 8183;
    case {3,4,10}
        overflow = 16385;
    case {5,6,7}
        overflow = 32759;
    otherwise
        warning('Unrecognized measuring mode: Using inf for overflow!');
end

end % overflowValue

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
function setPlotData(patch_obj,x_pos,y_pos,reflect)
%==========================================================================
% Func: setPlotData()
% Desc: Modifies the patch object's color data, as well as x and y data.
% Note: Coloring scheme can be tweaked for different outputs
%==========================================================================

% Specify color for each vertex (if reflect available, scale red channel)
colors = zeros(length(x_pos),3);      
if ~isempty(reflect), colors(:,1) = reflect./255; end
set(patch_obj,'FaceVertexCData',colors,'XData',x_pos,'YData',y_pos);
    
end % function setPlotData

%==========================================================================
%==========================================================================
function closeProgram(src,obj)
%==========================================================================
% Func: closeProgram()
% Desc: Indirectly stops the display and uninitializes the Sick
% Note: By setting the global keep_running to 0, this breaks out of the
%       while loop. This will execute when the figure window is closed. 
%==========================================================================
global keep_running;
keep_running = 0;

end % function closeProgram
