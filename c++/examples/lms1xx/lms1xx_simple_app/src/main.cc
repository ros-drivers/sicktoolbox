/*!
 * \file main.cc
 * \brief A simple application using the Sick LMS 1xx driver.
 *
 * Code by Jason C. Derenick and Christopher R. Mansley.
 * Contact jasonder(at)seas(dot)upenn(dot)edu
 *
 * The Sick LIDAR Matlab/C++ Toolbox
 * Copyright (c) 2009, Jason C. Derenick and Christopher R. Mansley
 * All rights reserved.
 *
 * This software is released under a BSD Open-Source License.
 * See http://sicktoolbox.sourceforge.net
 */

#include <string>
#include <iostream>
#include <sicklms1xx/SickLMS1xx.hh>

using namespace std;
using namespace SickToolbox;

int main(int argc, char* argv[])
{
  
  /*
   * Instantiate an instance
   */
  SickLMS1xx sick_lms_1xx;

  /*
   * Initialize the Sick LMS 2xx
   */
  try {
    sick_lms_1xx.Initialize();
  }

  catch(...) {
    cerr << "Initialize failed! Are you using the correct IP address?" << endl;
    return -1;
  }


  try {
    sick_lms_1xx.SetSickScanConfig();
  }

  catch(SickConfigException sick_exception) {
    std::cout << sick_exception.what() << std::endl;
  }
  
  catch(...) {
    cerr << "An Error Occurred!" << endl;
    return -1;
  }
  
  
  /*
   * Uninitialize the device
   */
  try {
    sick_lms_1xx.Uninitialize();
  }
  
  catch(...) {
    cerr << "Uninitialize failed!" << endl;
    return -1;
  }
  
  /* Success! */
  return 0;

}
    
