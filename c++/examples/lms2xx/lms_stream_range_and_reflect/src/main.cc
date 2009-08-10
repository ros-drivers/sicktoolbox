/*!
 * \file main.cc
 * \brief A simple program illustrating how to stream range
 *        and reflectivity returns from a Sick LMS 291-S14.
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

int main(int argc, char* argv[]) {

  string device_str;                      
  SickLMS::sick_lms_baud_t desired_baud = SickLMS::SICK_BAUD_38400;
  
  unsigned int num_range_values;
  unsigned int num_reflect_values;
  unsigned int range_values[SickLMS::SICK_MAX_NUM_MEASUREMENTS] = {0};
  unsigned int reflect_values[SickLMS::SICK_MAX_NUM_MEASUREMENTS] = {0};

  /* Check for a device path.  If it's not present, print a usage statement. */
  if ((argc != 2 && argc != 3) || (argc == 2 && strcasecmp(argv[1],"--help") == 0)) {
    cout << "Usage: lms_stream_range_and_reflect PATH [BAUD RATE]" << endl
	 << "Ex: lms_stream_range_and_reflect /dev/ttyUSB0 9600" << endl;
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
   * Instantiate driver instance
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

  /*
   * Ensure it is an LMS Fast model
   */
  if (sick_lms.IsSickLMSFast()) {

    /*
     * Grab range and reflectivity data from the Sick LMS Fast
     */        
    try {
      
      for (unsigned int i=0; i < 10; i++) {
	sick_lms.GetSickScan(range_values,reflect_values,num_range_values,num_reflect_values);
	cout << "Num. Range Vals: " << num_range_values << " Num. Reflect Vals: " << num_reflect_values << endl;	
      }
      
    }

    catch (...) {
      cerr << "An error occurred!" << endl;
    }

  }

  else {
    cerr << "Oops... Your Sick is NOT an LMS Fast!" << endl;
    cerr << "It doesn't support this kind of stream." << endl;
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
    
