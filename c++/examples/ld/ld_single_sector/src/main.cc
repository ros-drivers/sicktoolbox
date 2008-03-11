/*!
 * \file main.cc
 * \brief A simple application illustrating the use of
 *        the Sick LD C++ driver using a single sector.
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
#include <sickld-1.0/SickLD.hh>

/* Use the namespace */
using namespace std;
using namespace SickToolbox;

int main (int argc, char *argv[]) {

  /* A string for the IP address */
  string sick_ip_addr(DEFAULT_SICK_IP_ADDRESS);

  /* Check the num of args */
  if(argc > 2 || (argc == 2 && strcasecmp(argv[1],"--help") == 0)) {
    cerr << "Usage: ld_single_sector [SICK IP ADDRESS]" << endl
	      << "Ex. ld_single_sector 192.168.1.11" << endl;
    return -1;
  }
  
  /* Assign the IP address */
  if(argc == 2) {
    sick_ip_addr = argv[1];
  }

  /* Define the object */
  SickLD sick_ld(sick_ip_addr);

  /* Define the data buffers */
  double values[SickLD::SICK_MAX_NUM_MEASUREMENTS] = {0};
  unsigned int num_values = 0;

  /* Define the bounds for a single sector */
  double sector_start_ang = 90;
  double sector_stop_ang = 270;
  
  try {
  
    /* Attempt to initialize */
    sick_ld.Initialize();

    /* Set the desired sector configuration */
    sick_ld.SetSickTempScanAreas(&sector_start_ang,&sector_stop_ang,1);
    
    /* Print the sector configuration */
    sick_ld.PrintSickSectorConfig();
    
    /* Acquire some range measurements */
    for (unsigned int i = 0; i < 10; i++) {

      try {
      
	/* Here we only want the range values so the second arg is NULL */
	sick_ld.GetSickMeasurements(values,NULL,&num_values);
	cout << "\t  Num. Values: " << num_values << endl;    

      }

      /* Hope that the timeout isn't serious */
      catch(SickTimeoutException &sick_timeout_exception) {
	cerr << sick_timeout_exception.what() << endl;
      }

      /* Rethrow anything else */
      catch(...) {
	throw;
      }
      
    }

    /* Attempt to uninitialize */
    sick_ld.Uninitialize();

  }

  /* Catch any exceptions */
  catch(...) {
    cerr << "An error occurred!" << endl;
    return -1;
  }

  cout << "Done!!! :o)" << endl;
  
  /* Success !*/
  return 0;

}
