/*!
 * \file main.cc
 * \brief Illustrates how to acquire a measurements from the Sick
 *        LMS 2xx using the configured measuring mode.
 *
 * Note: This example should work for all Sick LMS 2xx models.
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

/* Implementation dependencies */
#include <string>
#include <vector>
#include <signal.h>
#include <iostream>
#include <sicklms-1.0/SickLMS.hh>
#include "gnuplot_i.hpp"

using namespace std;
using namespace SickToolbox;

bool running = true;
void sigintHandler(int signal);

int main(int argc, char * argv[]) {
  
  string device_str; // Device path of the Sick LMS 2xx
  SickLMS::sick_lms_baud_t desired_baud = SickLMS::SICK_BAUD_38400;
  
  /* Check for a device path.  If it's not present, print a usage statement. */
  if ((argc != 2 && argc != 3) || (argc == 2 && strcasecmp(argv[1],"--help") == 0)) {
    cout << "Usage: lms_plot_values PATH [BAUD RATE]" << endl
	 << "Ex: lms_plot_values /dev/ttyUSB0 9600" << endl;
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

  /* Signal handler */
  signal(SIGINT,sigintHandler);
  
  /* Define the plot */
  Gnuplot data_plot("points");
  string plot_label("Range");
  
  /* A vector to hold the values we are going to plot */
  vector<double> data_vector;  
  
  /* Instantiate the SickLMS class with the device path string. */
  SickLMS sick_lms(device_str);
  
  /* Define some buffers to hold the returned measurements */
  unsigned int values[SickLMS::SICK_MAX_NUM_MEASUREMENTS] = {0};
  unsigned int num_values = 0;

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
     * Check whether the device is returning reflectivity
     */
    if(sick_lms.GetSickMeasuringMode() == SickLMS::SICK_MS_MODE_REFLECTIVITY) {
      plot_label = "Reflectivity";
    }

    /*
     * Acquire measurements from Sick LMS 2xx and plot
     * them using gnuplot_i++
     */
    cout << "\tGrabbing 100 measurements..." << endl << endl;
    for (unsigned int i = 0; i < 100 && running; i++) {
      
      /* Acquire the most recent scan from the Sick */
      sick_lms.GetSickScan(values,num_values);
      
      /* Populate the data vector */
      for(unsigned int j = 0; j < num_values; j++) {
	data_vector.push_back((double)values[j]);
      }
      
      /* Plot the values */
      data_plot.plot_x(data_vector,plot_label.c_str());
      
      /* Sleep a bit (gnuplot likes this) */
      usleep(10000);
      
      /* Reset plot and vector */
      data_plot.reset_plot();
      data_vector.clear();
      
    }

  }

  /* Handle anything else */
  catch(...) {
    cerr << "An error occurred!"  << endl;
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

void sigintHandler(int signal){  running = false;  }
