/*!
 * \file main.cc
 * \brief Plots single and multi-pulse range values returned from
 *        the Sick LMS 1xx.
 *
 * Note: This example should work for all Sick LMS 1xx models.
 *
 * Code by Jason C. Derenick and Christopher R. Mansley.
 * Contact jasonder(at)seas(dot)upenn(dot)edu
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
#include <sicklms1xx/SickLMS1xx.hh>
#include "gnuplot_i.hpp"

using namespace std;
using namespace SickToolbox;

bool running = true;
void sigintHandler(int signal);

int main(int argc, char * argv[]) {
  
  /* Signal handler */
  signal(SIGINT,sigintHandler);
  
  /* Define the plot */
  Gnuplot data_plot("points");

  string plot_label("Range");
  
  /* A vector to hold the values we are going to plot */
  vector<double> data_vector;
  
  /* Instantiate the SickLMS2xx class with the device path string. */
  SickLMS1xx sick_lms_1xx;
  
  /* Define some buffers to hold the returned measurements */
  unsigned int range_vals[SickLMS1xx::SICK_MAX_NUM_MEASUREMENTS] = {0};
  unsigned int num_measurements = 0;

  /*
   * Initialize the Sick LMS 2xx
   */
  try {
    sick_lms_1xx.Initialize();
  }

  catch(...) {
    cerr << "Initialize failed! Are you using the correct device path?" << endl;
    return -1;
  }
  
  try {

    /*
     * Acquire measurements from Sick LMS 1xx and plot
     * them using gnuplot_i++
     */
    cout << "\tGrabbing 1000 measurements..." << endl << endl;
    for (unsigned int i = 0; i < 1000 && running; i++) {
      
      /* Acquire the most recent scan from the Sick */
      sick_lms_1xx.GetSickMeasurements(range_vals,NULL,NULL,NULL,num_measurements);
      
      /* Populate the data vector */
      for(unsigned int j = 0; j < num_measurements; j++) {
	data_vector.push_back((double)range_vals[j]);
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
    sick_lms_1xx.Uninitialize();
  }
  
  catch(...) {
    cerr << "Uninitialize failed!" << endl;
    return -1;
  }
  
  /* Success! */
  return 0;

}

void sigintHandler(int signal){  running = false;  }
