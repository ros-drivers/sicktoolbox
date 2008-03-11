/*!
 * \file main.cc
 * \brief A simple example illustrating how to acquire
 *        mean values from the Sick LMS 2xx.
 *
 * Code by Jason C. Derenick and Thomas H. Miller.
 * Contact derenick(at)lehigh(dot)edu
 *
 * The Sick LIDAR Matlab/C++ Toolbox
 * Copyright (c) 2008, Jason C. Derenick and Thomas H. Miller
 * All rights reserved.
 *
 * This software is released under a BSD Open-Source License.
 * See http://sicktoolbox.sourceforge.net
 */

#include <string>
#include <iostream>
#include <sicklms-1.0/SickLMS.hh>

using namespace std;
using namespace SickToolbox;

int main(int argc, char* argv[])
{
  
  string device_str;                      
  SickLMS::sick_lms_baud_t desired_baud = SickLMS::SICK_BAUD_38400;

  unsigned int values[SickLMS::SICK_MAX_NUM_MEASUREMENTS] = {0}; // Uses macro defined in SickLMS.hh
  unsigned int num_values;                                       // Holds the number of measurements returned

  /* Check for a device path.  If it's not present, print a usage statement. */
  if ((argc != 2 && argc != 3) || (argc == 2 && strcasecmp(argv[1],"--help") == 0)) {
    cout << "Usage: lms_mean_values PATH [BAUD RATE]" << endl
	 << "Ex: lms_mean_values /dev/ttyUSB0 9600" << endl;
    return -1;
  }

  /* Only device path is given */
  if (argc == 2) {
    device_str = argv[1];
  }

  /* Device path and baud are given */
  if (argc == 3) {    
    device_str = argv[1];
    if ((desired_baud = SickLMS::StringToSickBaud(argv[2])) == SickLMS::SICK_BAUD_UNKNOWN) {
      cerr << "Invalid baud value! Valid values are: 9600, 19200, 38400, and 500000" << endl;
      return -1;
    }
  }

  /*
   * Instantiate an instance
   */
  SickLMS sick_lms(device_str);

  try {

    /*
     * Initialize the LIDAR
     */
    sick_lms.Initialize(desired_baud);

    /*
     * Acquire a few scans from the Sick LMS
     */
    for (unsigned int i=0; i < 10; i++) {

      try {

	/*
	 * The first argument is an input to the driver
	 * telling it the number of raw scans over which
	 * the Sick should average. (i.e. here, the Sick
	 * will average over 5 scans before returning
	 * the resulting mean values)
	 */
	sick_lms.GetSickMeanValues(5,values,num_values);
	cout << "\t  Num. Values: " << num_values << endl;

      }

      /* Here we let the timeout slide and hope it isn't serious */
      catch(SickTimeoutException &sick_timeout_exception) {
	cerr << sick_timeout_exception.what() << endl;
      }

      /* Anything else is not ok, throw it away */
      catch(...) {
	throw;
      }
      
    }

    /*
     * Uninitialize the device
     */    
    sick_lms.Uninitialize();

  }

  /* Catch anything else and exit */ 
  catch(...) {
    cerr << "An error occurred!" << endl;
    return -1;
  }

  cout << "Done!!! :o)" << endl;
  
  return 0;

}
    
