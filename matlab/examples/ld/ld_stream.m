function ld_stream(sick_ip_addr,stream_type)
%==========================================================================
%==========================================================================
%
%  The Sick LIDAR Matlab/C++ Toolbox
%
%  File: ld_stream.m
%  Auth: Jason C. Derenick and Thomas H. Miller
%  Cont: derenick(at)lehigh(dot)edu
%  Date: 10 December 2007
%
%  In:   sick_ip_addr - IP address of the Sick LD unit
%        stream_type  - Type of stream (i.e. 'range' or 'range+reflect')
%  Out:  None
%
%  Desc:
%        This displays data from a Sick LD device associated with 
%        the given IP address.  If the device is configured with multiple
%        active sectors then this example will only plot data from
%        the first sector returned.
%
%        It uses sickld - a MEX function - to communicate with the Sick.
%        
%        To kill the data stream and uninitialize the Sick, simply 
%        close the graph window.
%
%        Usage:   ld_stream(SICK_IP,STREAM_TYPE)
%        Example: ld_stream('192.168.1.10','range+reflect')
%
%==========================================================================
global keep_running;

% Check args are given
if nargin ~= 2
    error('Invalid num of input args! Type "help ld_stream" for example usage.'); 
end

% Close all open figures
close all;

% At most four active sectors
ld_plot = cell(4,1);

% Initialize the device
sickld('init',sick_ip_addr);

disp(' '); disp('        *** CLOSE FIGURE TO QUIT ***'); disp(' ');

% Main process control loop
keep_running = 1;
while(keep_running)
    
    % Start data stream and grab measurements
    data = sickld(stream_type);
    if isempty(data), error('sickld grab failed!'); end
    
    % For each measuring sector generate a plot
    for i = 1:length(data)
    
        % If there isn't a figure then create one!
        if isempty(ld_plot{i})
            ld_plot{i} = setupAxes(stream_type,data(i));        
        end

        % Set the plot data
        setPlotData(data(i),ld_plot{i});

    end

    % Update the plot data
    drawnow; % Redraw the window
    
end

% Uninitialize the device
clear sickld;
close all;

end % ld_stream

%==========================================================================
%==========================================================================
function ld_plot = setupAxes(stream_type,data)
%==========================================================================
% Func: setupAxes()
% Desc: Creates an axes object for displaying the data stream
%==========================================================================
ld_plot.fig = ...
    figure('DeleteFcn',@closeProgram,'Name','Sick LD Data Stream');

% Setup the range subplot (after checking what type of plot to do)
if (strcmp(stream_type,'range'))
    ld_plot.subplot(1).h = subplot(1,1,1);  % For displaying range
else
    ld_plot.subplot(1).h = subplot(2,1,1);  % For displaying reflectivity
end

% Create plot title string
title_str = sprintf('Sick LD (Sector ID: %d)',data.id);

% Format the subplot
axis([1 length(data.range) 0 20]);
xlabel(sprintf('[%.1f, %.1f]',data.beg_ang,data.end_ang));
ylabel('Range (m)');
title(strcat(title_str,' - Range'));
set(gca,'box','on');
grid on;                        

% Create range line object
x_axis = 1:length(data.range);
y_axis = zeros(1,length(x_axis));
ld_plot.subplot(1).line = line('XData',x_axis,'YData',y_axis);

if (strcmp(stream_type,'range')), return; end

% Setup the reflectivity subplot
ld_plot.subplot(2).h = subplot(2,1,2);   % For displaying reflectivity data
axis([1 length(data.reflect) 0 700]);    % 700 is max reflectivity for the LD
title(strcat(title_str,' - Reflectivity'));
xlabel(sprintf('[%.1f, %.1f]',data.beg_ang,data.end_ang));
ylabel('\gamma');
set(gca,'box','on');
grid on;                        

% Create reflectivity line object
ld_plot.subplot(2).line = line('XData',x_axis,'YData',y_axis);

end % function setupAxes

%==========================================================================
%==========================================================================
function setPlotData(data,ld_plot)
%==========================================================================
% Func: setPlotData()
% Desc: Adjusts the line data to reflect the given measurement values
%==========================================================================

% Update the lines
set(ld_plot.subplot(1).line,'YData',data.range,'Color','b');

% Check whether we are streaming reflectivity
if ~isempty(data.reflect)
    set(ld_plot.subplot(2).line,'YData',data.reflect,'Color','r');
end
    
end % function setPlotData

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
