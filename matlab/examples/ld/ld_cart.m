function ld_cart(sick_ip_addr)
%==========================================================================
%==========================================================================
%
%  The Sick LIDAR Matlab/C++ Toolbox
%
%  File: ld_cart.m
%  Auth: Jason C. Derenick and Thomas H. Miller
%  Cont: derenick(at)lehigh(dot)edu
%  Date: 10 December 2007
%
%  In:   sick_ip_addr - IP address of the Sick LD unit
%  Out:  None
%
%  Desc:
%        This function uses the data streaming from the Sick to create a
%        "top-down" view of the surrounding environment.
%
%        To stop the Sick and the program, simply close the figure window.
%
%        Usage:   ld_cart(SICK_IP)
%        Example: ld_cart('192.168.0.12')
%
%==========================================================================
global keep_running

% Check args are given
if nargin ~= 1
    error('Invalid num of input args! Type "help ld_cart" for example usage.'); 
end

% Close all open figures
close all;

% Setup bounds for plot (meters)
ld_plot.x.ub =  10;
ld_plot.x.lb = -10;
ld_plot.y.ub =  10;
ld_plot.y.lb = -10;

% Initialize the Sick LD
sickld('init',sick_ip_addr);

% Initialize the plot figure
ld_plot = setupAxes(ld_plot);

disp(' '); disp('        *** CLOSE FIGURE TO QUIT ***'); disp(' ');

% Main process control loop
keep_running = 1;
while keep_running

    % Grab range and reflectivity values
    data = sickld('range+reflect');
    if isempty(data), error('sickld grab failed!'); end

    x_pos = [];
    y_pos = [];
    reflect = [];
    
    for i=1:length(data) % In case of multi-sector config

        % Get invalid range measurement mask 
        valid_indices = data(i).range > 0;

        % Extract only valid measurements
        data(i).range = data(i).range(valid_indices);
        data(i).reflect = data(i).reflect(valid_indices);

        % Generate angles associated w/ measurements
        theta = getThetas(data(i).beg_ang,data(i).end_ang,data(i).res_ang);
        theta = theta(valid_indices);

        % Convert polar to Cartesian coordinates (in Sick LD frame)
        [sec_x_pos sec_y_pos] = pol2cart(theta,data(i).range);
    
        % Include the points from this sector
        x_pos = [x_pos; sec_x_pos];
        y_pos = [y_pos; sec_y_pos];
        
        % Grab the reflectivity data
        reflect = [reflect; data(i).reflect];
        
    end
    
    % NOTE: We flip x & y to align Sick LD pts w/ Matlab plot frame
    setPlotData(y_pos,x_pos,reflect,ld_plot.p_obj);            
    drawnow; % Redraw the plot
        
end

% Uninitialize the device
clear sickld;

end % function ld_cart

%==========================================================================
%==========================================================================
function ld_plot = setupAxes(ld_plot)
%==========================================================================
% Func: setupAxes()
% Desc: Creates an axes object for displaying the data stream
%==========================================================================
figure('DeleteFcn',@closeProgram,'Name','Sick LD Cartesian Plot');
ld_plot.h = subplot(1,1,1);
axis(ld_plot.h,[ld_plot.x.lb ld_plot.x.ub ld_plot.y.lb ld_plot.y.ub]);
xlabel('Displacement (m)');
ylabel('Displacement (m)');
title('Sick LD Cartesian Plot');

% Use a single patch for plotting all values
ld_plot.p_obj = patch('Parent', ld_plot.h, ...
                      'linestyle', 'none', ...
                      'marker', '.', ...
                      'edgecolor', 'interp', ...
                      'facecolor', 'none', ...
                      'edgealpha', 0, ...
                      'facevertexcdata',[0 1 1]); 

% Generate the sick graphic                  
patch([0.06 0.06  0.0564  0.0460  0.0300  0.0104 -0.0104 -0.0300 -0.0460 -0.0564 -0.0600 -0.06], ...
      [0.07    0 -0.0205 -0.0386 -0.0520 -0.0591 -0.0591 -0.0520 -0.0386 -0.0205       0  0.07], ...
      [0 0 1]);
  
set(gca,'box','on');
grid on;                        

end % function setupAxes

%==========================================================================
%==========================================================================
function theta = getThetas(beg_ang,end_ang,res_ang)
%==========================================================================
% Func: setPlotData()
% Desc: Modifies the patch object's color data, as well as x and y data.
% Note: Coloring scheme can be tweaked for different outputs
%==========================================================================

% Scale red channel with reflectivity values
if beg_ang > end_ang, end_ang = end_ang+360; end
theta = mod((beg_ang:res_ang:end_ang)',360).*pi/180;

end % function setPlotData

%==========================================================================
%==========================================================================
function setPlotData(x_pos, y_pos,reflect,patchObj)
%==========================================================================
% Func: setPlotData()
% Desc: Modifies the patch object's color data, as well as x and y data.
% Note: Coloring scheme can be tweaked for different outputs
%==========================================================================

% Scale red channel with reflectivity values
scale = reflect./700;
colors = [scale zeros(length(reflect),2)];      
set(patchObj,'FaceVertexCData',colors,'XData',x_pos,'YData',y_pos);

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
