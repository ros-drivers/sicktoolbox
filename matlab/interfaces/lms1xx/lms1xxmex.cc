/*!
 * \file lms1xxmex.cc
 * \brief A Matlab mex interface for working w/ the Sick LMS 1xx
 *        family of laser range finders.
 *
 * Code by Jason C. Derenick and Christopher R. Mansley
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
#include <iostream>
#include <math.h>
#include "SickLMS1xx.hh"
#include "SickException.hh"
#include "mex.h"

/* Use the SickToolbox namespace */
using namespace SickToolbox;

/* Macros */
#define ARG_BUFF_LENGTH      (256)       ///< Max length (in bytes) of valid input arg
#define NUM_GRAB_STRUCT_KEYS   (4)

/** 
 * \fn initSick
 * \brief Initializes the device via the Sick LMS driver interface
 */
void initSick(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

/** 
 * \fn setScanFreqAndRes
 * \brief Sets the scan frequency and the angular resolution
 */
void setScanFreqAndRes(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

/** 
 * \fn grabSickVals
 * \brief Grabs the most recent measurements from the Sick LD via the driver interface
 */
void grabSickVals(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

/**
 * \fn validScanFormat
 * \brief Verifies the requested format is valid
 * \param scan_format A double input by the user, reflecting the desired data stream
 * \returns True if format is valid, False otherwise
 */
bool validScanFormat(double scan_format);

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

SickLMS1xx *sick_lms_1xx = NULL;
SickLMS1xx::sick_lms_1xx_scan_format_t curr_scan_format = SickLMS1xx::SICK_LMS_1XX_SCAN_FORMAT_UNKNOWN;

/* Main function (entry point) for MATLAB's use. */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{

  char cmd_buffer[ARG_BUFF_LENGTH] = {0}; // Buffer to hold command string

  /* Check for input arguments */
  if(!nrhs) {
    mexErrMsgTxt("Usage: sicklms1xx(cmd,[args]). Type \"help sicklms1xx\" for help and example usage.");
  }

  /* Grab the command */
  if(mxGetString(prhs[0],cmd_buffer,sizeof(cmd_buffer)) != 0) {
    mexErrMsgTxt("sicklms1xx: Could not read command.");
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
   * Set scan frequency and resolution
   */
  if(strcasecmp(cmd_buffer,"setfreqres") == 0) {
    setScanFreqAndRes(nlhs,plhs,nrhs,prhs);
    return;
  }
  
  /*
   * Grab measurements from the Sick
   */
  else if(strcasecmp(cmd_buffer,"grab") == 0) {
    grabSickVals(nlhs,plhs,nrhs,prhs);
    return;
  }

  /* Invalid command */
  else {
    mexErrMsgTxt("sicklms1xx: Unrecognized command!");
  }

}

/* Initialize a Sick LMS, and set up the corresponding SickDevice struct */
void initSick(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
      
  /* Set the default IP address */
  std::string sick_lms_1xx_ip_addr(DEFAULT_SICK_LMS_1XX_IP_ADDRESS);
  
  /* Check to see if an IP address was given */
  if(nrhs > 2) {
    mexErrMsgTxt("sicklms1xx: Invalid number of input args for init!\nExample usage: sicklms1xx('init') or sickld('init','192.168.0.11')");
  }
  else if(nrhs == 2) {

    /* Get the IP string */
    char arg_buffer[ARG_BUFF_LENGTH] = {0};
    if(mxGetString(prhs[1],arg_buffer,sizeof(arg_buffer)) != 0) {
      mexErrMsgTxt("sicklms1xx: Could not read argument!");
    }
    
    /* Set the IP address accordingly */
    sick_lms_1xx_ip_addr.assign(arg_buffer);
  }

  /* Check the number of lhs arguments */
  if(nlhs > 0) {
    mexErrMsgTxt("sicklms1xx: Invalid number of output args!\nsicklms1xx('init') doesn't return any values!");
  }

  /* Check whether its been allocated */
  if(sick_lms_1xx != NULL) {

    /* Check whether it is initialized */
    if(sick_lms_1xx->IsInitialized()) {
      mexWarnMsgTxt("Sick LMS 1xx is already initialized!\nClearing previous instance and re-initializing!");
    }

    /* Clear the previous instance */
    clearSick();
    
  }
  
  try {

    /* Instantiate the object */
    sick_lms_1xx = new SickLMS1xx(sick_lms_1xx_ip_addr);    

    /* Perform the initialization */
    mexPrintf("\n\tInitializing Sick LMS 1xx @ %s...\n",sick_lms_1xx_ip_addr.c_str());
    sick_lms_1xx->Initialize(false);    
    mexPrintf("\t\tDevice initialized!\n\n");

  }
  
  /* Catch a timeout */
  catch(SickTimeoutException &sick_timeout_exception) {      
    clearSick();
    mexErrMsgTxt("sicklms1xx: A timeout occurred! Are you using the correct IP address?");
  }  
  
  /* If there is an exception then quit */
  catch(...) {
    clearSick();
    mexErrMsgTxt("sicklms1xx: An error occurred!");
  }
  
  /* Register the exit callback */
  mexAtExit(mexExit);

}

/* Set scan frequency and resolution */
void setScanFreqAndRes(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

   /* Check whether device is already initialized */
   if(sick_lms_1xx == NULL || !sick_lms_1xx->IsInitialized()) {
     mexErrMsgTxt("sicklms1xx: Device is not initialized!");
   }

   SickLMS1xx::sick_lms_1xx_scan_freq_t desired_scan_freq = SickLMS1xx::SICK_LMS_1XX_SCAN_FREQ_UNKNOWN;
   SickLMS1xx::sick_lms_1xx_scan_res_t desired_scan_res = SickLMS1xx::SICK_LMS_1XX_SCAN_RES_UNKNOWN;     
   
   /* Make sure valid number of rhs arguments */
   if(nrhs != 3) {
     mexErrMsgTxt("sicklms1xx: Invalid number of input args for setfreqres!\nType \"help sicklms1xx\" for help.");
   }
   else {
     
     /* Grab the desired scan format */
     unsigned int m1, n1, m2, n2;
     m1 = mxGetM(prhs[1]);
     n1 = mxGetN(prhs[1]);

     m2 = mxGetM(prhs[2]);
     n2 = mxGetM(prhs[2]);
     
     /* Verify dimensions of input */
     if (m1 != n1 || n1 != 1 || m2 != n2 || n2 != 1) {
       mexErrMsgTxt("sicklms1xx: Argument(s) for setfreqres must be a scalar!\nType \"help sicklms1xx\" for help.");
     }

     /* Extract the parameters */
     double *scan_freq = mxGetPr(prhs[1]);
     double *scan_res = mxGetPr(prhs[2]);

     /* Ensure frequency is integer */
     if (*scan_freq != floor(*scan_freq)) {
       mexErrMsgTxt("sicklms1xx: Scan frequency for setfreqres must be 25 or 50!\nType \"help sicklms1xx\" for help.");
     }

     /* Convert the inputs to corresponding data-types */
     desired_scan_freq = sick_lms_1xx->IntToSickScanFreq((int)*scan_freq);
     desired_scan_res = sick_lms_1xx->DoubleToSickScanRes(*scan_res);     

     /* Ensure valid scan frequency */
     if (desired_scan_freq == SickLMS1xx::SICK_LMS_1XX_SCAN_FREQ_UNKNOWN) {
       mexErrMsgTxt("sicklms1xx: Scan frequency must be either 25Hz or 50Hz!\nType \"help sicklms1xx\" for help.");
     }

     /* Ensure valid scan resolution */
     if (desired_scan_res == SickLMS1xx::SICK_LMS_1XX_SCAN_RES_UNKNOWN) {
       mexErrMsgTxt("sicklms1xx: Scan resolution must be either 0.25 or 0.50 degrees!\nType \"help sicklms1xx\" for help.");
     }     

     /* Ensure valid scan freq and res pair */
     if (desired_scan_freq == SickLMS1xx::SICK_LMS_1XX_SCAN_FREQ_50 &&
	 desired_scan_res == SickLMS1xx::SICK_LMS_1XX_SCAN_RES_25) {
       mexErrMsgTxt("sicklms1xx: Sick LMS 1xx does not support the requested combination!\nType \"help sicklms1xx\" for help.");
     }
     
     /* Input looks valid so far */
     
   }

   /* Make sure valid number of lhs arguments */
   if (nlhs > 0) {
     mexErrMsgTxt("sicklms1xx: Invalid number of output args for setfreqres!\nType \"help sicklms1xx\" for help.");
   }
   
   /* Grab the latest values! */
   try {

     sick_lms_1xx->SetSickScanFreqAndRes(desired_scan_freq,desired_scan_res);
     
   }
   
   /* Catch other exceptions... */
   catch(...) {
     clearSick();
     mexErrMsgTxt("sicklms1xx: An error occurred!");
   }     
   
}


/* Grab Sick LMS measurements */
void grabSickVals(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

   /* Struct field names */
  const char *struct_keys[] = { "res",
				"fov",
				"range1",        // Angular resolution of the device
				"range2",        // Field of view of the Sick
				"reflect1",
				"reflect2" };

   /* Misc. buffers */
   int dims[2] = {0,1};
   unsigned int range_1_values[SickLMS1xx::SICK_MAX_NUM_MEASUREMENTS] = {0};
   unsigned int range_2_values[SickLMS1xx::SICK_MAX_NUM_MEASUREMENTS] = {0};
   unsigned int reflect_1_values[SickLMS1xx::SICK_MAX_NUM_MEASUREMENTS] = {0};
   unsigned int reflect_2_values[SickLMS1xx::SICK_MAX_NUM_MEASUREMENTS] = {0};   
   unsigned int num_measurements = 0;
   double scan_res = 0, scan_fov = 0;
  
   /* Declare mxArray ptrs */
   mxArray *mx_res_scalar = NULL;
   mxArray *mx_fov_scalar = NULL;
   mxArray *mx_range_1_array = NULL;
   mxArray *mx_range_2_array = NULL;  
   mxArray *mx_reflect_1_array = NULL; 
   mxArray *mx_reflect_2_array = NULL; 
   mxArray *mx_struct_array = NULL;
  
   /* Check whether device is already initialized */
   if(sick_lms_1xx == NULL || !sick_lms_1xx->IsInitialized()) {
     mexErrMsgTxt("sicklms1xx: Device is not initialized!");
   }
  
   SickLMS1xx::sick_lms_1xx_scan_format_t desired_scan_format = SickLMS1xx::SICK_LMS_1XX_SCAN_FORMAT_DIST_SINGLE_PULSE_REFLECT_NONE;

   /* Make sure valid number of rhs arguments */
   if(nrhs > 2) {
     mexErrMsgTxt("sicklms1xx: Invalid number of input args for grab!\nType \"help sicklms1xx\" for help.");
   }
   else if (nrhs == 2) {

     /* Grab the desired scan format */
     unsigned int m, n;
     m = mxGetM(prhs[1]);
     n = mxGetN(prhs[1]);

     /* Verify dimensions of input */
     if (m != n || n != 1) {
       mexErrMsgTxt("sicklms1xx: Argument for grab must be a scalar!\nType \"help sicklms1xx\" for help.");
     }

     /* Grab the desired scan format */
     double * scan_format_double = mxGetPr(prhs[1]);

     /* Verify the given format */
     if (!validScanFormat(*scan_format_double)) {
       mexErrMsgTxt("sicklms1xx: Invalid scan format for grab!\nType \"help sicklms1xx\" for help.");       
     }
     
     /* Scan format is valid! */
     desired_scan_format = (SickLMS1xx::sick_lms_1xx_scan_format_t)*scan_format_double;

   }

   /* Make sure valid number of lhs arguments */
   if (nlhs > 1) {
     mexErrMsgTxt("sicklms1xx: Invalid number of output args for grab!\nType \"help sicklms1xx\" for help.");
   }
   
   /* Grab the latest values! */
   try {
     
     /* Switch scan formats if necessary */
     if (curr_scan_format != desired_scan_format) {

       /* Kill the stream and switch formats... */
       mexPrintf("\n");       
       sick_lms_1xx->SetSickScanDataFormat(desired_scan_format);
       curr_scan_format = desired_scan_format;
       
     }
     
     /* If only single pulse dist measurements */
     if (curr_scan_format == SickLMS1xx::SICK_LMS_1XX_SCAN_FORMAT_DIST_SINGLE_PULSE_REFLECT_NONE) {
       
       sick_lms_1xx->GetSickMeasurements(range_1_values,
					 NULL,
					 NULL,
					 NULL,
					 num_measurements);

       /* Allocate numeric array for first-pulse range data */
       dims[0] = num_measurements;   
       if(!(mx_range_1_array = mxCreateNumericArray(2,dims,mxDOUBLE_CLASS,mxREAL))) {
	 mexErrMsgTxt("sicklms1xx: Failed to create numeric array!");
       }
       
     }
     
     /* If single pulse dist measurements w/ reflectivity */
     if (curr_scan_format == SickLMS1xx::SICK_LMS_1XX_SCAN_FORMAT_DIST_SINGLE_PULSE_REFLECT_8BIT ||
	 curr_scan_format == SickLMS1xx::SICK_LMS_1XX_SCAN_FORMAT_DIST_SINGLE_PULSE_REFLECT_16BIT) {
       
       sick_lms_1xx->GetSickMeasurements(range_1_values,
					 NULL,
					 reflect_1_values,
					 NULL,
					 num_measurements);

       /* Allocate numeric array for first-pulse range data */
       dims[0] = num_measurements;   
       if(!(mx_range_1_array = mxCreateNumericArray(2,dims,mxDOUBLE_CLASS,mxREAL))) {
	 mexErrMsgTxt("sicklms1xx: Failed to create numeric array!");
       }

       /* Allocate numeric array for first-pulse reflectivity data */
       if(!(mx_reflect_1_array = mxCreateNumericArray(2,dims,mxDOUBLE_CLASS,mxREAL))) {
	 mexErrMsgTxt("sicklms1xx: Failed to create numeric array!");
       }
       
     }       
     
     /* If only double pulse dist measurements */
     if (curr_scan_format == SickLMS1xx::SICK_LMS_1XX_SCAN_FORMAT_DIST_DOUBLE_PULSE_REFLECT_NONE) {
       
       sick_lms_1xx->GetSickMeasurements(range_1_values,
					 range_2_values,
					 NULL,
					 NULL,
					 num_measurements);

       /* Allocate numeric array for first-pulse range data */
       dims[0] = num_measurements;   
       if(!(mx_range_1_array = mxCreateNumericArray(2,dims,mxDOUBLE_CLASS,mxREAL))) {
	 mexErrMsgTxt("sicklms1xx: Failed to create numeric array!");
       }

       /* Allocate numeric array for first-pulse range data */
       if(!(mx_range_2_array = mxCreateNumericArray(2,dims,mxDOUBLE_CLASS,mxREAL))) {
	 mexErrMsgTxt("sicklms1xx: Failed to create numeric array!");
       }       
       
     }
     
     /* If double pulse dist measurements and reflectivity */       
     if (curr_scan_format == SickLMS1xx::SICK_LMS_1XX_SCAN_FORMAT_DIST_DOUBLE_PULSE_REFLECT_8BIT ||
	 curr_scan_format == SickLMS1xx::SICK_LMS_1XX_SCAN_FORMAT_DIST_DOUBLE_PULSE_REFLECT_16BIT) {
       
       sick_lms_1xx->GetSickMeasurements(range_1_values,
					 range_2_values,
					 reflect_1_values,
					 reflect_2_values,
					 num_measurements);

       /* Allocate numeric array for first-pulse range data */
       dims[0] = num_measurements;   
       if(!(mx_range_1_array = mxCreateNumericArray(2,dims,mxDOUBLE_CLASS,mxREAL))) {
	 mexErrMsgTxt("sicklms1xx: Failed to create numeric array!");
       }

       /* Allocate numeric array for second-pulse range data */
       if(!(mx_range_2_array = mxCreateNumericArray(2,dims,mxDOUBLE_CLASS,mxREAL))) {
	 mexErrMsgTxt("sicklms1xx: Failed to create numeric array!");
       }

       /* Allocate numeric array for first-pulse reflectivity */
       if(!(mx_reflect_1_array = mxCreateNumericArray(2,dims,mxDOUBLE_CLASS,mxREAL))) {
	 mexErrMsgTxt("sicklms1xx: Failed to create numeric array!");
       }

       /* Allocate numeric array for second-pulse reflectivity */
       if(!(mx_reflect_2_array = mxCreateNumericArray(2,dims,mxDOUBLE_CLASS,mxREAL))) {
	 mexErrMsgTxt("sicklms1xx: Failed to create numeric array!");
       }              
       
     }       
     
   }
   
   /* Catch any exceptions... */
   catch(...) {
     clearSick();
     mexErrMsgTxt("sicklms1xx: An error occurred!");
   }     

   /* Allocate the return struct */
   dims[0] = 1;
   if(!(mx_struct_array = mxCreateStructArray(2,dims,NUM_GRAB_STRUCT_KEYS,struct_keys))) {
     mexErrMsgTxt("sicklms1xx: Failed to create struct array!");
   }

   /* Create the return scalars */
   if(!(mx_res_scalar = mxCreateDoubleScalar(scan_res)) || !(mx_fov_scalar = mxCreateDoubleScalar(scan_fov))) {
     mexErrMsgTxt("sicklms1xx: Failed to create scalar!");
   }
   
   /* Construct return struct based upon requested stream */
   for(unsigned int i = 0; i < num_measurements; i++) {

     /* Assign first pulse range return */
     ((double*)mxGetPr(mx_range_1_array))[i] = range_1_values[i];
     mxSetField(mx_struct_array,0,struct_keys[0],mx_range_1_array);
     
     /* Assign second pulse range returns */
     if (mx_range_2_array != NULL) {
       ((double*)mxGetPr(mx_range_2_array))[i] = range_2_values[i];
       mxSetField(mx_struct_array,0,struct_keys[1],mx_range_2_array);       
     }

     /* Assign first pulse reflectivity returns */
     if (mx_reflect_1_array != NULL) {
       ((double*)mxGetPr(mx_reflect_1_array))[i] = reflect_1_values[i];
       mxSetField(mx_struct_array,0,struct_keys[2],mx_reflect_1_array);              
     }

     /* Assign first pulse reflectivity returns */
     if (mx_reflect_2_array != NULL) {
       ((double*)mxGetPr(mx_reflect_2_array))[i] = reflect_2_values[i];
       mxSetField(mx_struct_array,0,struct_keys[3],mx_reflect_2_array);                     
     }          
     
   }        

   /* Assign the output */
   plhs[0] = mx_struct_array;
   
}

/**
 * \fn validScanFormat
 * \brief Verifies the requested format is valid
 * \param scan_format A double input by the user, reflecting the desired data stream
 * \returns True if format is valid, False otherwise
 */
bool validScanFormat( double scan_format ) {

  if (scan_format != (double)SickLMS1xx::SICK_LMS_1XX_SCAN_FORMAT_DIST_SINGLE_PULSE_REFLECT_NONE &&
      scan_format != (double)SickLMS1xx::SICK_LMS_1XX_SCAN_FORMAT_DIST_SINGLE_PULSE_REFLECT_8BIT &&
      scan_format != (double)SickLMS1xx::SICK_LMS_1XX_SCAN_FORMAT_DIST_SINGLE_PULSE_REFLECT_16BIT &&      
      scan_format != (double)SickLMS1xx::SICK_LMS_1XX_SCAN_FORMAT_DIST_DOUBLE_PULSE_REFLECT_NONE &&
      scan_format != (double)SickLMS1xx::SICK_LMS_1XX_SCAN_FORMAT_DIST_DOUBLE_PULSE_REFLECT_8BIT &&
      scan_format != (double)SickLMS1xx::SICK_LMS_1XX_SCAN_FORMAT_DIST_DOUBLE_PULSE_REFLECT_16BIT) {

    return false;

  }

  /* Success! */
  return true;
  
}

/* Clear the Sick LD object */
void clearSick() {

  if(sick_lms_1xx) { 

    if(sick_lms_1xx->IsInitialized()) {
      
      try {
	mexPrintf("\n\tUninitializing Sick LMS 1xx...");
    	sick_lms_1xx->Uninitialize(false);
	mexPrintf("\n\t\tDevice uninitialized!\n");		
      }
      
      catch(...) {
    	mexWarnMsgTxt("sicklms1xx: An exception occurred! (Continuing to free Sick anyways)");
      }
      
    }

    delete sick_lms_1xx;
    sick_lms_1xx = NULL;
    
  }
  
}

/* Exit function for the MEX file */
void mexExit() {
  clearSick();
}
