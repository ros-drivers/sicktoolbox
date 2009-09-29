%==========================================================================
%==========================================================================
%
%  The Sick LIDAR Matlab/C++ Toolbox
%
%  File: sicklms.m
%  Auth: Jason C. Derenick and Thomas H. Miller
%  Cont: derenick(at)lehigh(dot)edu
%  Date: 11 January 2008
%
%  sicklms provides a Matlab mex interface for working with
%  Sick LMS 2xx model laser range finders.
%
%  COMMANDS:
%
%       1.) Initialize the Sick LMS 2xx
%       -------------------------------------------------------------------
%           RESULT = sicklms('init',DEV_PATH,SICK_BAUD);
%
%           Desc:
%               Before the interface can be used, the Sick must be 
%               first initialized via this command.
%               
%           Inputs:
%               DEV_PATH  - Refers to the Sick LMS 2xx device path.            
%               SICK_BAUD - Refers to the desired session baud rate.
%                           Valid values are: {9600,19200,38400,500000}
% 
%           Outputs:
%               RESULT    - Upon success, init command resturns a struct
%                           having three members:
%
%                               lms_fast  - Whether device is LMS Fast
%                               units_mm  - Whether units are mm (or cm)
%                               meas_mode - Measuring mode of device
%
%                           The latter two are boolean values indicating 
%                           whether the detected device is a Sick LMS Fast 
%                           (i.e. LMS 211-S14, LMS 221-S14, or LMS 291-S14) 
%                           and the measuring units respectively. If 
%                           units_mm is 0 then it can be assumed the units 
%                           are cm. The measuring mode indicates the 
%                           current measuring mode of the device. The 
%                           numerical values correspond to those appearing 
%                           on page 98 of the Sick LMS Telegram Listing.
%
%                           The init command will return an empty matrix
%                           upon failure.
%
%           Example Usage:
%               >> init_res = sicklms('init','/dev/ttyUSB0',500000);
%
%       2.) Uninitialize the Sick LMS 2xx
%       -------------------------------------------------------------------
%           To uninitialize the device, you simply need to clear the
%           mex file. In other words, enter the following:
%
%               >> clear sicklms;
%
%       3.) Setting the Sick LMS 2xx Variant
%       -------------------------------------------------------------------
%           sicklms('variant',SCAN_AREA,SCAN_RES);
%
%           Desc:
%               The total scan area and angular resolution of the Sick
%               LMS 2xx (excluding Sick LMS Fast models) can be changed 
%               without requiring any writes to EEPROM. The scan area
%               and resolution together are called the "variant."  To
%               set the variant, use this command.
%
%           Inputs:
%               SCAN_AREA - Refers to the Sick LMS 2xx scan FOV.
%                           Valid values are: {100,180}
%               SCAN_RES  - Refers to the desired resolution of each
%                           "scan" returned by the Sick LMS 2xx.
%                           Valid values are: {0.25,0.5,1.0}
%
%               NOTE: By default, the Sick LMS 2xx will use 180/0.5
%
%           Outputs:
%               None
%
%           NOTE: The method will throw a Matlab error if setting the
%                 variant fails for whatever reason.  As as a result,
%                 this command should always be used in a try-catch.
%
%           Example usage:
%               >> sicklms('variant',180,0.50);
%
%       4.) Grabbing Sick LMS 2xx Measurements
%       -------------------------------------------------------------------
%           DATA = sicklms('grab');
%
%           Desc:
%               Once initialized, data can then be requested from the
%               Sick LMS 2xx via the mex interface.  Upon the first 
%               invocation of the grab command, the low-level driver
%               will request a continuous stream of data from the Sick
%               LMS 2xx.  Each subsequent call to grab will the LATEST
%               scan buffered by the driver. 
%
%           Inputs:
%               None.
%
%           Outputs:
%               DATA - Upon success, the grab method will return a struct
%                      containing the most recent scan data. The struct
%                      has the following members:
%                      
%                         res      - Scan resolution {0.25,0.50,1.0} 
%                         fov      - Scan area/fov {90,100,180}
%                         range    - nx1 vector of range values*
%                         reflect  - nx1 vector of reflectivity values*
%
%                      *DATA.range will be empty if the device is set to
%                       stream only reflectivity values. DATA.reflect will
%                       empty if the device is streaming only range. For 
%                       LMS Fast models, DATA.range and DATA.reflect will
%                       both be populated.
%
%                      Upon failure, the grab command will return an empty
%                      matrix.  This indicates recoverable failure.
%
%           Example Usage:
%               >> data = sicklms('grab')
%
%       5.) Displaying Sick LMS 2xx Information
%       -------------------------------------------------------------------          
%           sicklms('info');
%
%           Desc:
%               Use this command to display Sick LMS 2xx status and
%               software information.
%
%           Inputs: 
%               None.
%
%           Outputs:
%               None.
%
%           Example Usage:               
%               >> sicklms('info')
%
%==========================================================================
