/*!
 * \file main.cc
 * \brief A simple program illustrating how to acquire a scan
 *        subrange from a Sick LMS 2xx.
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
  if ((argc != 2 && argc != 3) || (strcasecmp(argv[1],"--help") == 0)) {
    cout << "Usage: lms_subrange PATH [BAUD RATE]" << endl
	 << "Ex: lms_subrange /dev/ttyUSB0 9600" << endl;
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

  /*
   * Initialize the Sick LMS 2xx
   */
  try {
    sick_lms.Initialize(desired_baud);
  }

  catch(...) {
    cerr << "Initialize failed! Are you using the correct device path?" << endl;
    return -1;
  }  
  
  try {

    /*
     * Acquire a few scans from the Sick LMS
     */
    for (unsigned int i=0; i < 10; i++) {

      /*
       * Acquire the first ten measurements of a scan from the Sick
       */
      sick_lms.GetSickScanSubrange(1,10,values,num_values);
      cout << "\t  Num. Values: " << num_values << endl;
      
    }

  }

  catch(...) {
    cerr << "An error occurred!" << endl;
  }

  /*
   * Uninitialize the device
   */
  try {
    sick_lms.Uninitialize();
  }
  
  catch(...) {
    cerr << "Uninitialize failed!" << endl;
    return -1;
  }
  
  /* Success! */
  return 0;

}
    
