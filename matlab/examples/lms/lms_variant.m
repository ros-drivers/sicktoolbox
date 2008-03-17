function lms_variant(sick_dev_path,sick_baud)
%==========================================================================
%==========================================================================
%
%  The Sick LIDAR Matlab/C++ Toolbox
%
%  File: lms_variant.m
%  Auth: Jason C. Derenick and Thomas H. Miller
%  Cont: derenick(at)lehigh(dot)edu
%  Date: 11 January 2008
%
%  In:   sick_dev_path  - Sick device path
%        sick_baud_rate - Desired baud rate of comm session
%  Out:  None
%
%  Desc:
%        This function illustrates how to set the Sick LMS 2xx variant
%        (FOV and scan resolution).
%
%        Usage:   lms_variant(SICK_DEV_PATH,SICK_BAUD)
%        Example: lms_variant('/dev/ttyUSB0',500000)
%
%  NOTE: This example will not work for Sick LMS Fast models.  The 
%        variant command is not supported by these models.
% 
%==========================================================================

% Check args are given
if nargin ~= 2
    error('Invalid num of input args! Type "help lms_variant" for example usage.');
end

% Initialize the device
init_res = sicklms('init',sick_dev_path,sick_baud);
if isempty(init_res), error('sicklms init failed!'); end

% Check for LMS Fast
if init_res.lms_fast
   error('Oops! Sick LMS Fast models do not support the variant command.'); 
end

% Change the Sick LMS variant to 100/0.25
try
    sicklms('variant',100,0.25);  
catch
    disp('Failed to set variant to 100/0.25! (continuing anyways...)');
end

% Grab a scan and show the number of measurements is correct
data = sicklms('grab');
if isempty(data), error('sicklms grab failed!'); end
disp(data);

% Change the device variant to 180/0.5
try
    sicklms('variant',180,0.5);  
catch
    disp('Failed to set variant to 180/0.50! (continuing anyways...)');
end

data = sicklms('grab');
if isempty(data), error('sicklms grab failed!'); end
disp(data);

% Uninitialize the device
clear sicklms;

end % lms_variant
