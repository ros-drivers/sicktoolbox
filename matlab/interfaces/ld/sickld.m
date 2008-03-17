%==========================================================================
%==========================================================================
%
%  The Sick LIDAR Matlab/C++ Toolbox
%
%  File: sickld.m
%  Auth: Jason C. Derenick and Thomas H. Miller
%  Cont: derenick(at)lehigh(dot)edu
%  Date: 11 January 2008
%
%  sickld provides a Matlab mex interface for working with Sick LD 
%  laser range finders.
%
%  COMMANDS:
%
%       1.) Initialize the Sick LD
%       -------------------------------------------------------------------
%           sickld('init',[SICK_IP]);
%
%           Desc:
%               Before the interface can be used, the Sick must be 
%               first initialized via this command.
%               
%           Inputs:
%               SICK_IP - Sick LD ip address
% 
%           Outputs:
%               NONE
%
%           Example Usage:
%               >> sickld('init','192.168.1.11');
%
%       2.) Uninitialize the Sick LD
%       -------------------------------------------------------------------
%           To uninitialize the device, you simply need to clear the
%           mex file. In other words, enter the following:
%
%               >> clear sickld;
%
%       3.) Grabbing Sick LD Measurements
%       -------------------------------------------------------------------
%           DATA = sickld('range'); -OR- DATA = sickld('range+reflect');
%
%           Desc:
%               Once initialized, data can then be requested from the
%               Sick LD via the mex interface.  Upon the first invocation 
%               of the range (or range+reflect) command, the low-level 
%               driver will request a continuous stream of data from the 
%               Sick LD.  Each subsequent call to range (or range+reflect) 
%               will grab the LATEST scan buffered by the driver. 
%
%           Inputs:
%               None.
%
%           Outputs:
%               DATA - Upon success, the command will return an array of
%                      structs each containing the latest scan information
%                      for a single sector. Each struct in the array has
%                      the following structure:
%                      
%                        id        - Sector ID
%                        res_ang   - Angular resolution over sector
%                        beg_ang   - Angle corresponding to first value 
%                        end_ang   - Angle corresponding to last value                  
%                        beg_time  - Time corresponding to first value
%                        end_time  - Time corresponding to last value
%                        range     - nx1 vector of range values (meters)
%                        reflect   - nx1 vector of reflectivity values*
%
%                      *DATA.reflect will be empty if the device is 
%                       streaming only range.
%                       
%                      Upon failure, this command will return an empty
%                      matrix. This indicates recoverable failure.
%
%           Example Usage (for single sector config):
%               >> data = sickld('range')
%               >> disp(data)
%                         id: 0
%                    res_ang: 0.5000
%                    beg_ang: 90
%                    end_ang: 270
%                   beg_time: 17160
%                   end_time: 17210
%                      range: [361x1 double]
%                    reflect: []
%
%           Example Usage (for multi-sector config):
%               >> data = sickld('range')
%               >> disp(data(1))
%                         id: 0
%                    res_ang: 0.5000
%                    beg_ang: 90
%                    end_ang: 270
%                   beg_time: 23193
%                   end_time: 23244
%                      range: [361x1 double]
%                    reflect: []
%               >> disp(data(2))
%                         id: 2
%                    res_ang: 0.5000
%                    beg_ang: 355
%                    end_ang: 5
%                   beg_time: 23267
%                   end_time: 23270
%                      range: [21x1 double]
%                    reflect: []
%
%       4.) Displaying Sick LD Information
%       -------------------------------------------------------------------    
%           sickld('info');
%
%           Desc:
%               Use this command to display Sick LD status and
%               software information.
%
%           Inputs: 
%               None.
%
%           Outputs:
%               None.
%
%           Example Usage:               
%               >> sickld('info')
%
%==========================================================================
