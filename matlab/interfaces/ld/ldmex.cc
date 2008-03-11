/*!
 * \file ldmex.cc
 * \brief A Matlab mex interface for working w/ the Sick LD family
 *        of laser range finders.
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
#include <sstream>
#include <iostream>
#include "SickLD.hh"
#include "mex.h"

/* Macros */
#define ARG_BUFF_LENGTH (256) ///< Max length (in bytes) of valid input arg
#define NUM_STRUCT_KEYS   (8) ///< Number of keys in return struct

/* Use the SickToolbox namespace */
using namespace SickToolbox;

/** 
 * \fn initSick
 * \brief Initializes the device via the Sick LD driver interface.
 */
void initSick(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

/** 
 * \fn grabSickVals
 * \brief Grabs the most recent measurements from the Sick LD via the driver interface
 * \param grab_reflect Indicates whether to request reflectivity values as well as range
 */
mxArray * grabSickVals(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[], bool grab_reflect);

/**
 * \fn printSickInfo
 * \brief Prints the config/status information associated w/ Sick LD
 */
void printSickInfo(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

/**
 * \fn clearSick
 * \brief Clears an instance of the Sick LD device if one exists
 */
void clearSick();

/** 
 * \fn mexExit
 * \brief Called whenever the mex file is cleared or Matlab exits
 */
void mexExit();

/**
 * \var *sick_ld Global pointer to Sick LD driver instance
 *
 * NOTE: Global variables re-used in consecutive calls to function.
 *       Matlab does not erase non-mx global variables after returning from
 *       the mex file. 
 *
 *       For information on MATLAB's MEX memory management, look up
 *       "Advanced Topics" in the section "Creating C Language MEX-Files"
 *       in MATLAB's help.
 */
SickLD *sick_ld = NULL;

/* Main function (entry point) for Matlab */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{

  /* Buffer to hold command str */
  char cmd_buffer[ARG_BUFF_LENGTH] = {0};

  /* Check for input arguments */
  if(!nrhs) {
    mexErrMsgTxt("Usage: sickld(cmd,[args]). Type \"help sickld\" for help and example usage.");
  }
 
  /* Grab the command */
  if(mxGetString(prhs[0],cmd_buffer,sizeof(cmd_buffer)) != 0) {
    mexErrMsgTxt("sickld: Could not read command.");
  }

  /* Initialize the return struct */
  plhs[0] = NULL;
 
  /*
   * Initialize the device
   */
  if(strcasecmp(cmd_buffer,"init") == 0) {
    initSick(nlhs,plhs,nrhs,prhs);
    return;
  }
  
  /*
   * Grab values from range-only stream
   */
  else if(strcasecmp(cmd_buffer,"range") == 0) {
    plhs[0] = grabSickVals(nlhs,plhs,nrhs,prhs,false);
    return;
  }

  /*
   * Grab values from range w/ reflectivity stream
   */
  else if(strcasecmp(cmd_buffer,"range+reflect") == 0) {
    plhs[0] = grabSickVals(nlhs,plhs,nrhs,prhs,true);
    return;
  }

  /*
   * Print out the Sick LD status/config
   */
  else if(strcasecmp(cmd_buffer,"info") == 0) {
    printSickInfo(nlhs,plhs,nrhs,prhs);
    return;
  }

  /* Invalid command */
  else { 
    mexErrMsgTxt("sickld: Unrecognized command!");
  }

}

/* Initialize the driver and Sick */
void initSick(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
      
  /* Set the default IP address */
  std::string sick_ip_addr(DEFAULT_SICK_IP_ADDRESS);
  
  /* Check to see if an IP address was given */
  if(nrhs > 2) {
    mexErrMsgTxt("sickld: Invalid number of input args for init!\nExample usage: sickld('init') or sickld('init','192.168.0.11')");
  }
  else if(nrhs == 2) {

    /* Get the IP string */
    char arg_buffer[ARG_BUFF_LENGTH] = {0};
    if(mxGetString(prhs[1],arg_buffer,sizeof(arg_buffer)) != 0) {
      mexErrMsgTxt("sickld: Could not read argument!");
    }
    
    /* Set the IP address accordingly */
    sick_ip_addr.assign(arg_buffer);
  }

  /* Check the number of lhs arguments */
  if(nlhs > 0) {
    mexErrMsgTxt("sickld: Invalid number of output args!\nsickld('init') doesn't return any values!");
  }

  /* Check whether its been allocated */
  if(sick_ld) {

    /* Check whether it is initialized */
    if(sick_ld->IsInitialized()) {
      mexWarnMsgTxt("Sick LD is already initialized!\nClearing previous instance and re-initializing!");
    }

    /* Clear the previous instance */
    clearSick();
    
  }
  
  try {

    /* Instantiate the object */
    sick_ld = new SickLD(sick_ip_addr);    

    /* Perform the initialization */
    mexPrintf("\n\tInitializing Sick LD @ %s...\n",sick_ip_addr.c_str());
    sick_ld->Initialize();    
    mexPrintf("\t\tDevice initialized!\n");
    
  }
  
  /* Catch a timeout */
  catch(SickTimeoutException &sick_timeout_exception) {      
    clearSick();
    mexErrMsgTxt("A timeout occurred! Are you using the correct IP address?");
  }  
  
  /* If there is an exception then quit */
  catch(...) {
    clearSick();
    mexErrMsgTxt("An error occurred!");
  }
  
  /* Register the exit callback */
  mexAtExit(mexExit);
  
}

/* Grab Sick values */
mxArray * grabSickVals(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[], bool grab_reflect) {

  /* Struct field names */
  const char *struct_keys[] = { "id",         // Sector identity 
				"res_ang",    // Angular resolution of the sector
				"beg_ang",    // Angle at which sector starts
				"end_ang",    // Angle at which sector stops
				"beg_time",   // Time at which the first measurement occurred 
				"end_time",   // Time at which the last measurmement occurred
				"range",      // Range measurements 
				"reflect" };  // Reflectivity measurements

  /* Misc. ints */
  int dims[2] = {0,1};
  unsigned int num_active_sectors = 0;
  
  /* Declare mxArray ptrs */
  mxArray *mx_struct_array = NULL;  
  mxArray *mx_ran_array = NULL;
  mxArray *mx_ref_array = NULL;  
  mxArray *mx_sector_id_scalar = NULL;
  mxArray *mx_res_ang_scalar = NULL; 
  mxArray *mx_beg_ang_scalar = NULL; 
  mxArray *mx_end_ang_scalar = NULL; 
  mxArray *mx_beg_time_scalar = NULL; 
  mxArray *mx_end_time_scalar = NULL; 
  
  /* Create buffers to hold returned data from driver */
  double ran_values[SickLD::SICK_MAX_NUM_MEASUREMENTS] = {0};
  unsigned int ref_values[SickLD::SICK_MAX_NUM_MEASUREMENTS] = {0};

  /* Buffers to store additional information */
  unsigned int num_values[SickLD::SICK_MAX_NUM_SECTORS] = {0};
  unsigned int sector_ids[SickLD::SICK_MAX_NUM_SECTORS] = {0};
  unsigned int sector_offsets[SickLD::SICK_MAX_NUM_SECTORS] = {0};
  unsigned int sector_beg_times[SickLD::SICK_MAX_NUM_SECTORS] = {0};
  unsigned int sector_end_times[SickLD::SICK_MAX_NUM_SECTORS] = {0};
  double sector_res_angs[SickLD::SICK_MAX_NUM_SECTORS] = {0};
  double sector_beg_angs[SickLD::SICK_MAX_NUM_SECTORS] = {0};
  double sector_end_angs[SickLD::SICK_MAX_NUM_SECTORS] = {0};
  
  /* Check whether device is already initialized */
  if(!sick_ld || !sick_ld->IsInitialized()) {
    mexErrMsgTxt("sickld: Device is not initialized!");
  }
  
  /* Check the number of rhs (input) args */
  if(nrhs > 1) {
    
    std::stringstream str_stream;
    str_stream << "sickld: Invalid number of input args!\nExample usage: sickld('range";
    
    if(grab_reflect) {
      str_stream << "+reflect";
    }
    
    str_stream << "');";
    mexErrMsgTxt(str_stream.str().c_str());
    
  }

  /* Check the number of lhs (output) args */
  if(nlhs > 1) {

    std::stringstream str_stream;
    str_stream << "sickld: Invalid number of output args!\nExample usage: data = sickld('range";
    
    if(grab_reflect) {
      str_stream << "+reflect";
    }
    
    str_stream << "');";
    mexErrMsgTxt(str_stream.str().c_str());

  }
  
  try {
    
    /* Grab current measurements from the driver */
    sick_ld->GetSickMeasurements(ran_values,
				 grab_reflect ? ref_values : NULL,
				 num_values,
				 sector_ids,
				 sector_offsets,
				 sector_res_angs,
				 sector_beg_angs,
				 sector_end_angs,
				 sector_beg_times,
				 sector_end_times);      
    
  }

  /* If an exception occurs then bail */
  catch(...) {
    clearSick();
    mexErrMsgTxt("sickld: An error occurred!");    
  }
  
  /* Acquire the number of active sectors */
  num_active_sectors = sick_ld->GetSickNumActiveSectors();
  
  /* Allocate the return struct */
  dims[0] = num_active_sectors;
  if(!(mx_struct_array = mxCreateStructArray(2,dims,NUM_STRUCT_KEYS,struct_keys))) {
    mexErrMsgTxt("sickld: Failed to create struct array!");
  }
  
  /* Generate a return struct for each sector */
  for(unsigned int i = 0; i < num_active_sectors; i++) {
    
    dims[0] = num_values[i];
    
    /* Allocate the array */
    if(!(mx_ran_array = mxCreateNumericArray(2,dims,mxDOUBLE_CLASS,mxREAL))) {
      mexErrMsgTxt("sickld: Failed to create numeric array!");
    }

    /* Populate the range array */    
    for (unsigned int j = 0; j < num_values[i]; j++) {
      ((double*)mxGetPr(mx_ran_array))[j] = ran_values[sector_offsets[i]+j];
    }        
    
    /* If reflectivity values are requested */
    if(grab_reflect) {      

      /* Allocate the reflectivity array */
      if(!(mx_ref_array = mxCreateNumericArray(2,dims,mxDOUBLE_CLASS,mxREAL))) {
	mexErrMsgTxt("sickld: Failed to create numeric array!");
      }

      /* Populate the reflectivity array */    
      for (unsigned int j = 0; j < num_values[i]; j++) {
	((double*)mxGetPr(mx_ref_array))[j] = (double)ref_values[sector_offsets[i]+j];
      }
      
    }
    
    /* Create the return scalars */
    if(!(mx_sector_id_scalar = mxCreateDoubleScalar(sector_ids[i])) ||
       !(mx_res_ang_scalar = mxCreateDoubleScalar(sector_res_angs[i])) ||
       !(mx_beg_ang_scalar = mxCreateDoubleScalar(sector_beg_angs[i])) ||
       !(mx_end_ang_scalar = mxCreateDoubleScalar(sector_end_angs[i])) ||
       !(mx_beg_time_scalar = mxCreateDoubleScalar(sector_beg_times[i])) ||
       !(mx_end_time_scalar = mxCreateDoubleScalar(sector_end_times[i]))) {
      mexErrMsgTxt("sickld: Failed to create scalar!");
    }
    
    /* Set the pointers in the return struct */
    mxSetField(mx_struct_array,i,struct_keys[0],mx_sector_id_scalar);
    mxSetField(mx_struct_array,i,struct_keys[1],mx_res_ang_scalar);
    mxSetField(mx_struct_array,i,struct_keys[2],mx_beg_ang_scalar);
    mxSetField(mx_struct_array,i,struct_keys[3],mx_end_ang_scalar);
    mxSetField(mx_struct_array,i,struct_keys[4],mx_beg_time_scalar);
    mxSetField(mx_struct_array,i,struct_keys[5],mx_end_time_scalar);
    mxSetField(mx_struct_array,i,struct_keys[6],mx_ran_array);
    mxSetField(mx_struct_array,i,struct_keys[7],mx_ref_array);
    
  }

  /* Return the pointer to the struct */
  return mx_struct_array;
  
}

/* Prints the current sick config/status to std out */
void printSickInfo(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

  /* Check whether device is already initialized */
  if(!sick_ld || !sick_ld->IsInitialized()) {
    mexErrMsgTxt("sickld: Device is not initialized!");
  }
  
  /* Check the number of arguments */
  if(nrhs > 1) {
    mexErrMsgTxt("sickld: Invalid number of input args!\nExample usage: sickld('info')");
  }

  /* Check the number of lhs arguments */
  if(nlhs > 0) {
    mexErrMsgTxt("sickld: Invalid number of output args!\n Info doesn't return any values!");
  }
  
  /* Dump the status/config std out */
  mexPrintf(sick_ld->GetSickStatusAsString().c_str());
  mexPrintf(sick_ld->GetSickIdentityAsString().c_str());
  mexPrintf(sick_ld->GetSickGlobalConfigAsString().c_str());
  mexPrintf(sick_ld->GetSickEthernetConfigAsString().c_str());
  mexPrintf(sick_ld->GetSickSectorConfigAsString().c_str());
  
}

/* Clear the Sick LD object */
void clearSick() {
  
  if(sick_ld) { 

    if(sick_ld->IsInitialized()) {
    
      try {
	sick_ld->Uninitialize();
      }

      catch(...) {
	mexWarnMsgTxt("sickld: An error occurred! (Continuing to free Sick anyways)");
      }

    }
      
    delete sick_ld;
    sick_ld = NULL;
    
  }
  
}

/* Exit function for the MEX file */
void mexExit() {
  clearSick();
}

