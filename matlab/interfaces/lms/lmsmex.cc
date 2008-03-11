/*!
 * \file lmsmex.cc
 * \brief A Matlab mex interface for working w/ the Sick LMS 2xx
 *        family of laser range finders.
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
#include <iostream>
#include "SickLMS.hh"
#include "SickException.hh"
#include "mex.h"

/* Use the SickToolbox namespace */
using namespace SickToolbox;

/* Macros */
#define ARG_BUFF_LENGTH      (256)       ///< Max length (in bytes) of valid input arg
#define NUM_INIT_STRUCT_KEYS   (3)
#define NUM_GRAB_STRUCT_KEYS   (4)

/** 
 * \fn initSick
 * \brief Initializes the device via the Sick LMS driver interface
 */
void initSick(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

/**
 * \fn setSickVariant
 * \brief Attempts to set the device variant via the Sick LMS drive interface
 */
void setSickVariant(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

/** 
 * \fn grabSickVals
 * \brief Grabs the most recent measurements from the Sick LD via the driver interface
 * \param grab_reflect Indicates whether to request reflectivity values as well as range
 */
void grabSickVals(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

/**
 * \fn printSickInfo
 * \brief Prints the config/status information associated w/ Sick LMS
 */
void printSickInfo(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

/**
 * \fn clearSick
 * \brief Uninitializes one or more Sicks
 */
void clearSick();

/** 
 * \fn mexExit
 * \brief Called whenever the mex file is cleared or Matlab exits
 */
void mexExit();

SickLMS *sick_lms = NULL;

/* Main function (entry point) for MATLAB's use. */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{

  char cmd_buffer[ARG_BUFF_LENGTH] = {0}; // Buffer to hold command string

  /* Check for input arguments */
  if(!nrhs) {
    mexErrMsgTxt("Usage: sicklms(cmd,[args]). Type \"help sicklms\" for help and example usage.");
  }

  /* Grab the command */
  if(mxGetString(prhs[0],cmd_buffer,sizeof(cmd_buffer)) != 0) {
    mexErrMsgTxt("sicklms: Could not read command.");
  }

  /* Initialize the return values */
  plhs[0] = NULL;

  /*
   * Initialize the device
   */
  if(strcasecmp(cmd_buffer,"init") == 0) {
    initSick(nlhs,plhs,nrhs,prhs);
    return;
  }

  /*
   * Set device variant FOV w/ resolution
   */
  else if(strcasecmp(cmd_buffer,"variant") == 0) {
    setSickVariant(nlhs,plhs,nrhs,prhs);
    return;
  }
  
  /*
   * Grab measurements from the Sick
   */
  else if(strcasecmp(cmd_buffer,"grab") == 0) {
    grabSickVals(nlhs,plhs,nrhs,prhs);
    return;
  }

  /*
   * Print out the Sick LMS status/config
   */
  else if(strcasecmp(cmd_buffer,"info") == 0) {
    printSickInfo(nlhs,plhs,nrhs,prhs);
    return;
  }
  
  /* Invalid command */
  else {
    mexErrMsgTxt("sicklms: Unrecognized command!");
  }

}

/* Initialize a Sick LMS, and set up the corresponding SickDevice struct */
void initSick(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

  /* Device path */
  std::string sick_dev_path;

  /* Struct field names */
  const char *struct_keys[] = { "lms_fast",    // Indicates whether device is LMS Fast
				"units_mm",
                                "meas_mode" };  // Indicates whether units are in (mm)

  /* Misc. bools */
  int dims[2] = {0,1};
  bool lms_fast, units_mm;
  SickLMS::sick_lms_measuring_mode_t meas_mode;
  
  /* Declare mxArrays */
  mxArray *mx_struct_array = NULL;
  mxArray *mx_units_mm_logical = NULL;
  mxArray *mx_lms_fast_logical = NULL; 
  mxArray *mx_meas_mode_array = NULL;
  
  /* Make sure each of the args is given */
  if(nrhs != 3) {
    mexErrMsgTxt("sicklms: Invalid number of input args for init!\nType \"help sicklms\" for help.");
  }

  /* Check the number of lhs arguments */
  if(nlhs > 1) {
    mexErrMsgTxt("Invalid number of output args.\nType \"help sicklms\" for help.");
  }
  
  /* Get the device path */
  char arg_buffer[ARG_BUFF_LENGTH] = {0};
  if(mxGetString(prhs[1],arg_buffer,sizeof(arg_buffer)) != 0) {
    mexErrMsgTxt("sicklms: Could not read argument!");
  }

  /* Assign the device path */
  sick_dev_path.assign(arg_buffer);
  
  /* Check whether its been allocated */
  if(sick_lms) {
    
    /* Check whether it is initialized */
    if(sick_lms->IsInitialized()) {
      mexWarnMsgTxt("Sick LMS is already initialized!\nClearing previous instance and re-initializing!");
    }
    
    /* Clear the previous instance */
    clearSick();
    
  }
  
  /* Get the desired baud rate */
  SickLMS::sick_lms_baud_t sick_baud;
  if((sick_baud = SickLMS::IntToSickBaud((int)*((double *)mxGetPr(prhs[2])))) == SickLMS::SICK_BAUD_UNKNOWN) {
    mexErrMsgTxt("sicklms: Invalid baud rate! Valid values are 9600, 19200, 38400, and 500000.");
  }
  
  try {
    
    /* Instantiate the object */
    sick_lms = new SickLMS(sick_dev_path);    
    
    /* Perform the initialization */
    mexPrintf("\n\tInitializing Sick LMS @ %s...\n",sick_dev_path.c_str());
    sick_lms->Initialize(sick_baud);
    mexPrintf("\t\tDevice initialized! (%s)\n\n",SickLMS::SickBaudToString(sick_baud).c_str());

    /* Get the Sick LMS device type */
    lms_fast = sick_lms->IsSickLMSFast();
    
    /* Get the measuring units */
    units_mm = sick_lms->GetSickMeasuringUnits() == SickLMS::SICK_MEASURING_UNITS_MM;

    /* Get the Sick LMS measuring mode */
    meas_mode = sick_lms->GetSickMeasuringMode();
    
  }
  
  /* Catch an I/O exception */
  catch(SickIOException &sick_io_exception) {      
    clearSick();
    mexErrMsgTxt("sicklms: An I/O error occurred! Are you using the correct device path?");
  }  
  
  /* If there is an exception then quit */
  catch(...) {
    clearSick();
    mexErrMsgTxt("sicklms: An error occurred!");
  }

  /* Allocate the return struct */
  dims[0] = 1;
  if(!(mx_struct_array = mxCreateStructArray(2,dims,NUM_INIT_STRUCT_KEYS,struct_keys))) {
    mexErrMsgTxt("sicklms: Failed to create struct array!");
  }

  /* Create the return scalars */
  if(!(mx_lms_fast_logical = mxCreateLogicalScalar(lms_fast)) ||
     !(mx_units_mm_logical = mxCreateLogicalScalar(units_mm)) ||
     !(mx_meas_mode_array = mxCreateDoubleScalar(meas_mode))) {
    mexErrMsgTxt("sicklms: Failed to create scalar!");
  }

  /* Set struct elements */
  mxSetField(mx_struct_array,0,struct_keys[0],mx_lms_fast_logical);
  mxSetField(mx_struct_array,0,struct_keys[1],mx_units_mm_logical);
  mxSetField(mx_struct_array,0,struct_keys[2],mx_meas_mode_array);
  
  /* Register the exit callback */
  if(mexAtExit(mexExit) != 0) {
    mexErrMsgTxt("sicklms: Failed to register exit function!");
  }

  /* Assign the return struct */
  plhs[0] = mx_struct_array;

}

/* Initialize a Sick LMS, and set up the corresponding SickDevice struct */
void setSickVariant(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

  /* Has the device object been allocated */
  if(!sick_lms || !sick_lms->IsInitialized()) {
    mexErrMsgTxt("Sick LMS is not initialized. Please initialize the device first.");
  }

  /* Make sure each of the args is given */
  if(nrhs != 3) {
    mexErrMsgTxt("sicklms: Invalid number of input args for variant!\nType \"help sicklms\" for help.");
  }

  /* Check the number of lhs arguments */
  if(nlhs > 0) {
    mexErrMsgTxt("Invalid number of output args.\nType \"help sicklms\" for help.");
  }
  
  /* Check whether device is an LMS Fast */
  if (sick_lms->IsSickLMSFast()) {
    mexErrMsgTxt("Variant command is not supported by this Sick model (LMS Fast)! (Ignoring request)");
  }
  
  /* Get the desired scan angle/fov */
  int scan_angle = (int)*((double *)mxGetPr(prhs[1]));
  SickLMS::sick_lms_scan_angle_t sick_scan_angle;
  if((sick_scan_angle = SickLMS::IntToSickScanAngle(scan_angle)) == SickLMS::SICK_SCAN_ANGLE_UNKNOWN) {
    mexErrMsgTxt("sicklms: Invalid scan angle (FOV)! Valid values are 100 and 180.");
  }

  /* Get the desired scan resolution */
  double scan_resolution = *((double *)mxGetPr(prhs[2]));
  SickLMS::sick_lms_scan_resolution_t sick_scan_resolution;
  if((sick_scan_resolution = SickLMS::DoubleToSickScanResolution(scan_resolution)) == SickLMS::SICK_SCAN_RESOLUTION_UNKNOWN) {
    mexErrMsgTxt("sicklms: Invalid scan resolution! Valid values are 0.25, 0.50, and 1.0.");
  }
  
  try {

    /* Set the desired Sick LMS variant */
    mexPrintf("\tAttempting to set variant to %d/%1.2f...\n",scan_angle,scan_resolution);
    sick_lms->SetSickVariant(sick_scan_angle,sick_scan_resolution);
    mexPrintf("\t\tVariant set!\n");
    
  }

  /* Catch config exception */
  catch(SickConfigException &sick_config_exception) {      
    mexErrMsgTxt("sicklms: A config error occurred! Variant may not be supported by this model!");
  }  

  /* If there is an exception then quit */
  catch(...) {
    clearSick();
    mexErrMsgTxt("sicklms: An error occurred!");
  }
    
}

/* Grab Sick LMS measurements */
void grabSickVals(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

  /* Struct field names */
  const char *struct_keys[] = { "res",         // Angular resolution of the device
				"fov",         // Field of view of the Sick
                                "range",
                                "reflect" };
  
  /* Misc. buffers */
  int dims[2] = {0,1};
  unsigned int pri_values[SickLMS::SICK_MAX_NUM_MEASUREMENTS] = {0};
  unsigned int sec_values[SickLMS::SICK_MAX_NUM_MEASUREMENTS] = {0};
  unsigned int num_pri_values = 0, num_sec_values = 0;
  double scan_res = 0, scan_fov = 0;
  
  /* Declare mxArray ptrs */
  mxArray *mx_pri_array = NULL;
  mxArray *mx_sec_array = NULL;  
  mxArray *mx_res_scalar = NULL; 
  mxArray *mx_fov_scalar = NULL; 
  mxArray *mx_struct_array = NULL;
  
  /* For current Sick measuring mode */
  sick_lms_measuring_mode_t measuring_mode = SickLMS::SICK_MS_MODE_UNKNOWN;

  /* Check whether device is already initialized */
  if(!sick_lms || !sick_lms->IsInitialized()) {
    mexErrMsgTxt("sicklms: Device is not initialized!");
  }
  
  /* Make sure each of the args is given */
  if(nrhs != 1) {
    mexErrMsgTxt("sicklms: Invalid number of input args for grab!\nType \"help sicklms\" for help.");
  }
  
  /* Check the output args */
  if(nlhs > 2 && sick_lms->IsSickLMSFast()) {
    mexErrMsgTxt("Invalid number of output args.\nType \"help sicklms\" for help.");
  }

  /* Check the output args */
  if(nlhs > 1 && sick_lms->IsSickLMSFast()) {
    mexErrMsgTxt("Invalid number of output args.\nType \"help sicklms\" for help.");
  }

  try {

    /* If the device is an LMS Fast grab both range and reflectivity */
    if(sick_lms->IsSickLMSFast()) {
      sick_lms->GetSickScan(pri_values,sec_values,num_pri_values,num_sec_values);
    }
    else {
      sick_lms->GetSickScan(pri_values,num_pri_values);
    }
    
    /* Grab the current measuring mode */
    measuring_mode = sick_lms->GetSickMeasuringMode();
    
    /* Grab current scan resolution w/ fov */
    scan_res = sick_lms->GetSickScanResolution();
    scan_fov = sick_lms->GetSickScanAngle();
    
  }

  catch(...) {
    clearSick();
    mexErrMsgTxt("sicklms: An error occurred!");
  }

  /* Allocate the return struct */
  dims[0] = 1;
  if(!(mx_struct_array = mxCreateStructArray(2,dims,NUM_GRAB_STRUCT_KEYS,struct_keys))) {
    mexErrMsgTxt("sicklms: Failed to create struct array!");
  }
  
  /* Create the return scalars */
  if(!(mx_res_scalar = mxCreateDoubleScalar(scan_res)) || !(mx_fov_scalar = mxCreateDoubleScalar(scan_fov))) {
    mexErrMsgTxt("sicklms: Failed to create scalar!");
  }

  /* Setup scalars in the return struct */
  mxSetField(mx_struct_array,0,struct_keys[0],mx_res_scalar);
  mxSetField(mx_struct_array,0,struct_keys[1],mx_fov_scalar);
  
  /* Allocate numeric array for scan data */
  dims[0] = num_pri_values;   
  if(!(mx_pri_array = mxCreateNumericArray(2,dims,mxDOUBLE_CLASS,mxREAL))) {
    mexErrMsgTxt("sicklms: Failed to create numeric array!");
  }

  /* Copy measurements into return array */
  for(unsigned int i = 0; i < num_pri_values; i++) {
    ((double*)mxGetPr(mx_pri_array))[i] = pri_values[i];
  }        
  
  /* Check whether to grab reflectivity */
  if(sick_lms->IsSickLMSFast()) {

    /* Create array to hold reflectivity values */
    dims[0] = num_sec_values;   
    if(!(mx_sec_array = mxCreateNumericArray(2,dims,mxDOUBLE_CLASS,mxREAL))) {
      mexErrMsgTxt("sicklms: Failed to create numeric array!");
    }     

    /* Populate the array */
    for(unsigned int i = 0; i < num_sec_values; i++) {
      ((double*)mxGetPr(mx_sec_array))[i] = sec_values[i];
    }        

    /* Return both range and reflectivity values */
    mxSetField(mx_struct_array,0,struct_keys[2],mx_pri_array);
    mxSetField(mx_struct_array,0,struct_keys[3],mx_sec_array);
    
  }
  else {

    if(measuring_mode == SickLMS::SICK_MS_MODE_REFLECTIVITY) {
      /* Sick is returning reflectivity only */      
      mxSetField(mx_struct_array,0,struct_keys[3],mx_pri_array);
    }
    else {
      /* Sick is returning range only */
      mxSetField(mx_struct_array,0,struct_keys[2],mx_pri_array);
    }

  }
    
  /* Assign the output */
  plhs[0] = mx_struct_array;

}

/* Prints the current sick config/status to std out */
void printSickInfo(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

  /* Check whether device is already initialized */
  if(!sick_lms || !sick_lms->IsInitialized()) {
    mexErrMsgTxt("sicklms: Device is not initialized!");
  }
  
  /* Check the number of arguments */
  if(nrhs > 1) {
    mexErrMsgTxt("sicklms: Invalid number of input args!\nType \"help sicklms\" for help.");
  }

  /* Check the number of lhs arguments */
  if(nlhs > 0) {
    mexErrMsgTxt("sicklms: Invalid number of output args!\nType \"help sicklms\" for help.");
  }
  
  /* Dump the status/config */
  mexPrintf(sick_lms->GetSickStatusAsString().c_str());
  mexPrintf(sick_lms->GetSickSoftwareVersionAsString().c_str());
  
}

/* Clear the Sick LD object */
void clearSick() {

  if(sick_lms) { 

    if(sick_lms->IsInitialized()) {
      
      try {
    	sick_lms->Uninitialize();
      }
      
      catch(...) {
    	mexWarnMsgTxt("sicklms: An exception occurred! (Continuing to free Sick anyways)");
      }
      
    }

    delete sick_lms;
    sick_lms = NULL;
    
  }
  
}

/* Exit function for the MEX file */
void mexExit() {
  clearSick();
}
