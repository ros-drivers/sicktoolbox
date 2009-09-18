/*!
 * \file SickLMS1xx.cc
 * \brief Implements the SickLMS1xx driver class.
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

/* Auto-generated header */
#include "SickConfig.hh"

/* Implementation dependencies */
#include <string>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <sys/socket.h>       // for socket function definitions
#include <arpa/inet.h>        // for sockaddr_in, inet_addr, and htons
#include <sys/ioctl.h>        // for using ioctl functionality for the socket input buffer
#include <unistd.h>           // for select functionality (e.g. FD_SET, etc...)
#include <sys/types.h>        // for fd data types
#include <sys/time.h>         // for select timeout parameter
#include <fcntl.h>            // for getting file flags
#include <pthread.h>          // for POSIX threads
#include <sstream>            // for parsing ip addresses
#include <vector>             // for returning the results of parsed strings
#include <errno.h>            // for timing connect()

#include "SickLMS1xx.hh"
#include "SickLMS1xxMessage.hh"
#include "SickLMS1xxBufferMonitor.hh"
#include "SickLMS1xxUtility.hh"   
#include "SickException.hh"

/* Associate the namespace */
namespace SickToolbox {

  /**
   * \brief A standard constructor
   * \param sick_ip_address The ip address of the Sick LD
   * \param sick_tcp_port The TCP port associated w/ the Sick LD server
   */
  SickLMS1xx::SickLMS1xx( const std::string sick_ip_address, const uint16_t sick_tcp_port ) :
    SickLIDAR< SickLMS1xxBufferMonitor, SickLMS1xxMessage >( ),
    _sick_ip_address(sick_ip_address),
    _sick_tcp_port(sick_tcp_port)
  {
    memset(&_sick_scan_config,0,sizeof(sick_lms_1xx_scan_config_t));
  }

  /**
   * A standard destructor
   */
  SickLMS1xx::~SickLMS1xx( ) { }

  /**
   * \brief Initializes the driver and syncs it with Sick LMS 1xx unit. Uses flash params.
   */
  void SickLMS1xx::Initialize( ) throw( SickIOException, SickThreadException, SickTimeoutException, SickErrorException ) {

    std::cout << "\t*** Attempting to initialize the Sick LMS 1xx..." << std::endl; 

    try {
      
      /* Attempt to connect to the Sick LD */
      std::cout << "\tAttempting to connect to Sick LMS 1xx @ " << _sick_ip_address << ":" << _sick_tcp_port << std::endl;
      _setupConnection();
      std::cout << "\t\tConnected to Sick LMS 1xx!" << std::endl;

      /* Start the buffer monitor */
      std::cout << "\tAttempting to start buffer monitor..." << std::endl;
      _startListening();
      std::cout << "\t\tBuffer monitor started!" << std::endl;
    
      /* Ok, lets sync the driver with the Sick */
      std::cout << "\tSyncing driver with Sick..." << std::endl;
      _getSickScanConfig();
      std::cout << "\t\tSuccess!" << std::endl;
      
    }
    
    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    catch(SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }

    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }

    catch(...) {
      std::cerr << "SickLMS1xx::Initialize - Unknown exception!" << std::endl;
      throw;
    }
    
    //std::cout << "\t\tSynchronized!" << std::endl;
    _sick_initialized = true;
    
    _printInitFooter();
    
    /* Success */
  }

  /**
   * \brief Get the scan configuration of the Sick LMS 1xx
   */
  void SickLMS1xx::SetSickScanConfig( ) throw( SickTimeoutException, SickIOException, SickConfigException ) {

    /* Allocate a single buffer for payload contents */
    uint8_t payload_buffer[SickLMS1xxMessage::MESSAGE_PAYLOAD_MAX_LENGTH+1] = {0};

    std::cout << "\t*** Attempting to configure device..." << std::endl;
    
    /* Set the command type */
    payload_buffer[0]  = 's';
    payload_buffer[1]  = 'M';
    payload_buffer[2]  = 'N';
    
    payload_buffer[3]  = ' ';

    /* Set the command */
    payload_buffer[4]  = 'm';
    payload_buffer[5]  = 'L';
    payload_buffer[6]  = 'M';
    payload_buffer[7]  = 'P';
    payload_buffer[8]  = 's';
    payload_buffer[9]  = 'e';
    payload_buffer[10] = 't';
    payload_buffer[11] = 's';
    payload_buffer[12] = 'c';
    payload_buffer[13] = 'a';
    payload_buffer[14] = 'n';
    payload_buffer[15] = 'c';
    payload_buffer[16] = 'f';
    payload_buffer[17] = 'g';

    payload_buffer[18] = ' ';    

    /* Desired scanning frequency */
    payload_buffer[19] = '+';    
    payload_buffer[20] = '2';
    payload_buffer[21] = '5';
    payload_buffer[22] = '0';
    payload_buffer[23] = '0';

    payload_buffer[24] = ' ';    

    /* Desired number of segments (always 1) */
    payload_buffer[25] = '+';
    payload_buffer[26] = '1';

    payload_buffer[27] = ' ';    
    
    /* Desired angular resolution */
    payload_buffer[28] = '+';   
    payload_buffer[29] = '5';
    payload_buffer[30] = '0';
    payload_buffer[31] = '0';
    payload_buffer[32] = '0';

    payload_buffer[33] = ' ';

    /* Desired starting angle */
    payload_buffer[34] = '-';    
    payload_buffer[35] = '4';
    payload_buffer[36] = '4';
    payload_buffer[37] = '2';
    payload_buffer[38] = '2';
    payload_buffer[39] = '0';    
    payload_buffer[40] = '0';
    
    payload_buffer[41] = ' ';

    /* Desired stopping angle */
    payload_buffer[42] = '+';    
    payload_buffer[43] = '2';
    payload_buffer[44] = '2';    
    payload_buffer[45] = '4';
    payload_buffer[46] = '3';
    payload_buffer[47] = '0';
    payload_buffer[48] = '0';
    payload_buffer[49] = '0';        
    
    /* Construct command message */
    SickLMS1xxMessage send_message(payload_buffer,50);

    /* Setup container for recv message */
    SickLMS1xxMessage recv_message;

    /* Set access mode and send configuration */
    try {

      /* Set the authorized client access mode */
      if (!_setAuthorizedClientAccessMode()) {
	throw SickIOException("SickLMS1xx::SetSickScanConfig: _setAuthorizedClientAccessMode failed!");	
      }
      
      _sendMessageAndGetReply(send_message, recv_message, "sAN", "mLMPsetscancfg");

    }
        
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle write buffer exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS1xx::_sendMessageAndGetReply: Unknown exception!!!" << std::endl;
      throw;
    }
    
    /* Reset the buffer (not necessary, but its better to do so just in case) */
    memset(payload_buffer,0,SickLMS1xxMessage::MESSAGE_PAYLOAD_MAX_LENGTH+1);
  
    /* Extract the message payload */
    recv_message.GetPayload(payload_buffer);
    
    /* Check if it worked... */
    if (payload_buffer[19] != '0') {
	throw SickConfigException("SickLMS1xx::SetSickScanConfig: " + _intToSickConfigErrorStr(atoi((char *)&payload_buffer[19])));	      
    }

    /* Update the scan configuration! */
    try {

      _getSickScanConfig();
      
    }

    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle write buffer exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS1xx::SetSickScanConfig: Unknown exception!!!" << std::endl;
      throw;
    }

    /* Success */
    std::cout << "\t\tDevice configured!" << std::endl << std::endl;

    _printSickScanConfig();
    
  }

  /**
   * \brief Tear down the connection between the host and the Sick LD
   */
  void SickLMS1xx::Uninitialize( ) throw( SickIOException, SickTimeoutException, SickErrorException, SickThreadException ){

    /* Ensure the device has been initialized */
    if (!_sick_initialized) {
      throw SickIOException("SickLMS1xx::Uninitialize: Device NOT Initialized!!!");
    }

    std::cout << std::endl << "\t*** Attempting to uninitialize the Sick LMS 1xx..." << std::endl; 
  
    /* If necessary, tell the Sick LD to stop streaming data */
    try {
      
      //std::cout << "\tSetting Sick LD to idle mode..." << std::endl;
      //_setSickSensorModeToIdle();
      //std::cout << "\t\tSick LD is now idle!" << std::endl;

      /* Clear any signals that were set */
      //SetSickSignals();

      /* Attempt to cancel the buffer monitor */
      std::cout << "\tAttempting to cancel buffer monitor..." << std::endl;
      _stopListening();
      std::cout << "\t\tBuffer monitor canceled!" << std::endl;
    
      /* Attempt to close the tcp connection */
      std::cout << "\tClosing connection to Sick LMS 1xx..." << std::endl;
      _teardownConnection();

    }
           
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle I/O exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* Handle a returned error code */
    catch (SickErrorException &sick_error_exception) {
      std::cerr << sick_error_exception.what() << std::endl;
      throw;
    }

    /* Handle a returned error code */
    catch (SickThreadException &sick_thread_exception) {
      std::cerr << sick_thread_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS::_setSickSensorMode: Unknown exception!!!" << std::endl;
      throw;
    }  

    std::cout << "\t\tConnection closed!" << std::endl;

    std::cout << "\t*** Uninit. complete - Sick LMS 1xx is now offline!" << std::endl; 

    /* Mark the device as uninitialized */
    _sick_initialized = false;
  
    /* Success! */
  }

  /**
   * \brief Establish a TCP connection to the unit
   */
  void SickLMS1xx::_setupConnection( ) throw( SickIOException, SickTimeoutException ) {

    /* Create the TCP socket */
    if ((_sick_fd = socket(PF_INET,SOCK_STREAM,IPPROTO_TCP)) < 0) {
      throw SickIOException("SickLMS1xx::_setupConnection: socket() failed!");
    }

    /* Initialize the buffer */
    memset(&_sick_inet_address_info,0,sizeof(struct sockaddr_in));
  
    /* Setup the Sick LD inet address structure */
    _sick_inet_address_info.sin_family = AF_INET;                                  // Internet protocol address family
    _sick_inet_address_info.sin_port = htons(_sick_tcp_port);                      // TCP port number
    _sick_inet_address_info.sin_addr.s_addr = inet_addr(_sick_ip_address.c_str()); // Convert ip string to numerical address

    try {

      /* Set to non-blocking so we can time connect */
      _setNonBlockingIO();
    
      /* Try to connect to the Sick LD */
      int conn_return;
      if ((conn_return = connect(_sick_fd,(struct sockaddr *)&_sick_inet_address_info,sizeof(struct sockaddr_in))) < 0) {

	/* Check whether it is b/c it would block */
	if (errno != EINPROGRESS) {	
	  throw SickIOException("SickLMS1xx::_setupConnection: connect() failed!");
	}

	/* Use select to wait on the socket */
	int valid_opt = 0;
	int num_active_files = 0;
	struct timeval timeout_val;                                  // This structure will be used for setting our timeout values
	fd_set file_desc_set;                                        // File descriptor set for monitoring I/O
    
	/* Initialize and set the file descriptor set for select */
	FD_ZERO(&file_desc_set);
	FD_SET(_sick_fd,&file_desc_set);

	/* Setup the timeout structure */
	memset(&timeout_val,0,sizeof(timeout_val));                  // Initialize the buffer
	timeout_val.tv_usec = DEFAULT_SICK_LMS_1XX_CONNECT_TIMEOUT;  // Wait for specified time before throwing a timeout
      
	/* Wait for the OS to tell us that the connection is established! */
	num_active_files = select(getdtablesize(),0,&file_desc_set,0,&timeout_val);
      
	/* Figure out what to do based on the output of select */
	if (num_active_files > 0) {
	
	  /* This is just a sanity check */
	  if (!FD_ISSET(_sick_fd,&file_desc_set)) {
  	    throw SickIOException("SickLMS1xx::_setupConnection: Unexpected file descriptor!");
	  }	  

	  /* Check for any errors on the socket - just to be sure */
	  socklen_t len = sizeof(int);
	  if (getsockopt(_sick_fd,SOL_SOCKET,SO_ERROR,(void*)(&valid_opt),&len) < 0) { 	    
  	    throw SickIOException("SickLMS1xx::_setupConnection: getsockopt() failed!");
	  } 

	  /* Check whether the opt value indicates error */
	  if (valid_opt) { 
	    throw SickIOException("SickLMS1xx::_setupConnection: socket error on connect()!");
	  }
	  
  	}
	else if (num_active_files == 0) {
	
	  /* A timeout has occurred! */
	  throw SickTimeoutException("SickLMS1xx::_setupConnection: select() timeout!");	

	}
	else {
	
	  /* An error has occurred! */
	  throw SickIOException("SickLMS1xx::_setupConnection: select() failed!");	

	}

      }

      /* Restore blocking IO */
      _setBlockingIO();	
	
    }

    catch(SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }

    catch(SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    catch(...) {
      std::cerr << "SickLMS1xx::_setupConnection - Unknown exception occurred!" << std::endl;
      throw;
    }

    /* Success */
  }

  
   /**
    * \brief Teardown TCP connection to Sick LD
    */
  void SickLMS1xx::_teardownConnection( ) throw( SickIOException ) {
     
     /* Close the socket! */
     if (close(_sick_fd) < 0) {
       throw SickIOException("SickLMS1xx::_teardownConnection: close() failed!");
     }
     
   }

  /**
   * \brief Get the status of the Sick LD
   */
  void SickLMS1xx::_getSickStatus( sick_lms_1xx_status_t &sick_status, bool &temp_status ) throw( SickTimeoutException, SickIOException ) {

    /* Allocate a single buffer for payload contents */
    uint8_t payload_buffer[SickLMS1xxMessage::MESSAGE_PAYLOAD_MAX_LENGTH] = {0};

    /* Set the command type */
    payload_buffer[0] = 's';
    payload_buffer[1] = 'R';
    payload_buffer[2] = 'N';
    
    payload_buffer[3] = ' ';

    /* Set the command */
    payload_buffer[4] = 'S';
    payload_buffer[5] = 'T';
    payload_buffer[6] = 'l';
    payload_buffer[7] = 'm';
    payload_buffer[8] = 's';

    /* Construct command message */
    SickLMS1xxMessage send_message(payload_buffer,9);

    /* Setup container for recv message */
    SickLMS1xxMessage recv_message;

    /* Send message and get reply using parent's method */
    try {
      
      _sendMessageAndGetReply(send_message, recv_message, "sRA", "STlms");

    }
        
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle write buffer exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS1xx::_sendMessageAndGetReply: Unknown exception!!!" << std::endl;
      throw;
    }
    
    /* Reset the buffer (not necessary, but its better to do so just in case) */
    memset(payload_buffer,0,9);
  
    /* Extract the message payload */
    recv_message.GetPayload(payload_buffer);

    sick_status = _intToSickStatus(atoi((char *)&payload_buffer[10])); // (sick_lms_1xx_status_t)payload_buffer[11];
    temp_status = (bool)atoi((char *)&payload_buffer[12]);

    /* Success */

  }

  /**
   * \brief Get the scan configuration of the Sick LMS 1xx
   */
  void SickLMS1xx::_getSickScanConfig( ) throw( SickTimeoutException, SickIOException ) {
				      

    /* Allocate a single buffer for payload contents */
    uint8_t payload_buffer[SickLMS1xxMessage::MESSAGE_PAYLOAD_MAX_LENGTH+1] = {0};

    /* Set the command type */
    payload_buffer[0]  = 's';
    payload_buffer[1]  = 'R';
    payload_buffer[2]  = 'N';
    
    payload_buffer[3]  = ' ';

    /* Set the command */
    payload_buffer[4]  = 'L';
    payload_buffer[5]  = 'M';
    payload_buffer[6]  = 'P';
    payload_buffer[7]  = 's';
    payload_buffer[8]  = 'c';
    payload_buffer[9]  = 'a';
    payload_buffer[10] = 'n';
    payload_buffer[11] = 'c';
    payload_buffer[12] = 'f';
    payload_buffer[13] = 'g';    

    /* Construct command message */
    SickLMS1xxMessage send_message(payload_buffer,14);

    /* Setup container for recv message */
    SickLMS1xxMessage recv_message;

    /* Send message and get reply using parent's method */
    try {
      
      _sendMessageAndGetReply(send_message, recv_message, "sRA", "LMPscancfg");

    }
        
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle write buffer exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS1xx::_sendMessageAndGetReply: Unknown exception!!!" << std::endl;
      throw;
    }
    
    /* Reset the buffer (not necessary, but its better to do so just in case) */
    memset(payload_buffer,0,14);
  
    /* Extract the message payload */
    recv_message.GetPayloadAsCStr((char *)payload_buffer);

    /* Utility variables */
    uint32_t scan_freq = 0, scan_res = 0;
    uint32_t start_angle = 0, stop_angle = 0;
    
    /*
     * Grab the scanning frequency
     */
    const char * token = NULL;
    if ((token = strtok((char *)&payload_buffer[15]," ")) == NULL) {
      throw SickIOException("SickLMS1xx::_getSickConfig: strtok() failed!");
    }

    if (sscanf(token,"%x",&scan_freq) == EOF) {
      throw SickIOException("SickLMS1xx::_getSickConfig: sscanf() failed!");
    }

    scan_freq = sick_lms_1xx_to_host_byte_order(scan_freq);

    /* Ignore the number of segments value (its always 1 for the LMS 1xx) */
    if ((token = strtok(NULL," ")) == NULL) {
      throw SickIOException("SickLMS1xx::_getSickConfig: strtok() failed!");
    }

    /*
     * Grab the angular resolution
     */    
    if ((token = strtok(NULL," ")) == NULL) {
      throw SickIOException("SickLMS1xx::_getSickConfig: strtok() failed!");
    }
    
    if (sscanf(token,"%x",&scan_res) == EOF) {
      throw SickIOException("SickLMS1xx::_getSickConfig: sscanf() failed!");
    }
    
    scan_res = sick_lms_1xx_to_host_byte_order(scan_res);

    /*
     * Grab the start angle
     */    
    if ((token = strtok(NULL," ")) == NULL) {
      throw SickIOException("SickLMS1xx::_getSickConfig: strtok() failed!");
    }
    
    if (sscanf(token,"%x",&start_angle) == EOF) {
      throw SickIOException("SickLMS1xx::_getSickConfig: sscanf() failed!");
    }
    
    start_angle = sick_lms_1xx_to_host_byte_order(start_angle);

    /*
     * Grab the stop angle
     */    
    if ((token = strtok(NULL," ")) == NULL) {
      throw SickIOException("SickLMS1xx::_getSickConfig: strtok() failed!");
    }
    
    if (sscanf(token,"%x",&stop_angle) == EOF) {
      throw SickIOException("SickLMS1xx::_getSickConfig: sscanf() failed!");
    }
    
    stop_angle = sick_lms_1xx_to_host_byte_order(stop_angle);

    /*
     * Assign the config values!
     */
    _sick_scan_config.sick_scan_freq = scan_freq;
    _sick_scan_config.sick_scan_res = scan_res;
    _sick_scan_config.sick_start_angle = start_angle;
    _sick_scan_config.sick_stop_angle = stop_angle;
    
    /* Success */

  }  

  /**
   * \brief Login as an authorized client
   */
  bool SickLMS1xx::_setAuthorizedClientAccessMode() throw( SickTimeoutException, SickIOException ) {

    /* Allocate a single buffer for payload contents */
    uint8_t payload_buffer[SickLMS1xxMessage::MESSAGE_PAYLOAD_MAX_LENGTH+1] = {0};
    
    /* Set the command type */
    payload_buffer[0]  = 's';
    payload_buffer[1]  = 'M';
    payload_buffer[2]  = 'N';
    
    payload_buffer[3]  = ' ';

    /* Set the command */
    payload_buffer[4]  = 'S';
    payload_buffer[5]  = 'e';
    payload_buffer[6]  = 't';
    payload_buffer[7]  = 'A';
    payload_buffer[8]  = 'c';
    payload_buffer[9]  = 'c';
    payload_buffer[10] = 'e';
    payload_buffer[11] = 's';
    payload_buffer[12] = 's';
    payload_buffer[13] = 'M';
    payload_buffer[14] = 'o';
    payload_buffer[15] = 'd';
    payload_buffer[16] = 'e';    

    payload_buffer[17] = ' ';
    
    /* Set as authorized client */
    payload_buffer[18] = '0';
    payload_buffer[19] = '3';

    payload_buffer[20] = ' ';

    /* Encoded value for client */
    payload_buffer[21] = 'F';
    payload_buffer[22] = '4';
    payload_buffer[23] = '7';
    payload_buffer[24] = '2';
    payload_buffer[25] = '4';
    payload_buffer[26] = '7';
    payload_buffer[27] = '4';
    payload_buffer[28] = '4';

    /* Construct command message */
    SickLMS1xxMessage send_message(payload_buffer,29);

    /* Setup container for recv message */
    SickLMS1xxMessage recv_message;

    /* Send message and get reply using parent's method */
    try {
      
      _sendMessageAndGetReply(send_message, recv_message, "sAN", "SetAccessMode");

    }
        
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout_exception) {
      std::cerr << sick_timeout_exception.what() << std::endl;
      throw;
    }
    
    /* Handle write buffer exceptions */
    catch (SickIOException &sick_io_exception) {
      std::cerr << sick_io_exception.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS1xx::_setAuthorizedClientAccessMode: Unknown exception!!!" << std::endl;
      throw;
    }
    
    /* Reset the buffer (not necessary, but its better to do so just in case) */
    memset(payload_buffer,0,29);
    
    /* Extract the message payload */
    recv_message.GetPayload(payload_buffer);

    /* Check Response */
    if (payload_buffer[18] != '1') {
      std::cerr << "SickLMS1xx::_setAuthorizedClientAccessMode: Setting Access Mode Failed!" << std::endl;    
      return false;
    }

    /* Success! Woohoo! */
    return true;
    
  }
  
  /**
   * \brief Sends a message and searches for the reply with given command type and command
   * \param &send_message The message to be sent to the Sick LMS 2xx unit
   * \param &recv_message The expected message reply from the Sick LMS
   * \param reply_command_code The expected command code for the recv_message
   * \param reply_command The expected command for the recv_message
   * \param timeout_value The epoch to wait before considering a sent frame lost (in usecs)
   * \param num_tries The number of times to send the message in the event the LMS fails to reply
   */
  void SickLMS1xx::_sendMessageAndGetReply( const SickLMS1xxMessage &send_message,
					    SickLMS1xxMessage &recv_message,
					    const std::string reply_command_type,
					    const std::string reply_command,
					    const unsigned int timeout_value,
					    const unsigned int num_tries ) throw( SickIOException, SickTimeoutException ) {

    /* Construct the expected string */
    std::string expected_str = reply_command_type + " " + reply_command;
    
    try {

      /* Send a message and get reply using parent's method */
      SickLIDAR< SickLMS1xxBufferMonitor, SickLMS1xxMessage >::_sendMessageAndGetReply(send_message,recv_message,(uint8_t *)expected_str.c_str(),expected_str.length(),DEFAULT_SICK_LMS_1XX_BYTE_TIMEOUT,timeout_value,num_tries);

    }
    
    /* Handle a timeout! */
    catch (SickTimeoutException &sick_timeout) {
      throw;
    }
    
    /* Handle write buffer exceptions */
    catch (SickIOException &sick_io_error) {
      std::cerr << sick_io_error.what() << std::endl;
      throw;
    }
    
    /* A safety net */
    catch (...) {
      std::cerr << "SickLMS1xx::_sendMessageAndGetReply: Unknown exception!!!" << std::endl;
      throw;
    }
    
  }
  
  /**
   * \brief Converts int to status
   * \param status integer corresponding to sick status
   */
  sick_lms_1xx_status_t SickLMS1xx::_intToSickStatus( const int status ) const
  {
    switch(status) {
    case 1:
      return SICK_LMS_1XX_STATUS_INITIALIZATION;
    case 2:
      return SICK_LMS_1XX_STATUS_CONFIGURATION;
    case 3:
      return SICK_LMS_1XX_STATUS_IDLE;
    case 4:
      return SICK_LMS_1XX_STATUS_ROTATED;
    case 5:
      return SICK_LMS_1XX_STATUS_IN_PREP;
    case 6:
      return SICK_LMS_1XX_STATUS_READY;
    case 7:
      return SICK_LMS_1XX_STATUS_READY_FOR_MEASUREMENT;
    default:
      return SICK_LMS_1XX_STATUS_UNDEFINED;
    }
  }

  /** Utility function to convert config error int to str */
  std::string SickLMS1xx::_intToSickConfigErrorStr( const int error ) const {

    switch(error) {
    case 1:
      return "Invalid frequency";
    case 2:
      return "Invalid angular resolution";
    case 3:
      return "Invalid frequency and angular resolution";
    case 4:
      return "Invalid scan area";
    case 5:
      return "Other error";
    default:
      return "Unrecognized error code";
    }
  
  }

  /**
   * \brief Prints Sick LMS 1xx scan configuration
   */
  void SickLMS1xx::_printSickScanConfig( ) const {
  
    std::cout << "\t========= Sick Scan Config =========" << std::endl;
    std::cout << "\tScan Frequency: " << ((double)_sick_scan_config.sick_scan_freq)/100 << "(Hz)" << std::endl;  
    std::cout << "\tScan Resolution: " << ((double)_sick_scan_config.sick_scan_res)/10000 << " (deg)" << std::endl;  
    std::cout << "\tScan Area: " << "[" << ((double)_sick_scan_config.sick_start_angle)/10000 << "," << ((double)_sick_scan_config.sick_stop_angle)/10000 << "]" << std::endl;
    std::cout << "\t====================================" << std::endl;
    std::cout << std::flush;
  }
  
  /**
   * \brief Prints the initialization footer.
   */
  void SickLMS1xx::_printInitFooter( ) const {

    std::cout << "\t*** Init. complete: Sick LMS 1xx is online and ready!" << std::endl; 
    std::cout << "\tScan Frequency: " << ((double)_sick_scan_config.sick_scan_freq)/100 << "(Hz)" << std::endl;  
    std::cout << "\tScan Resolution: " << ((double)_sick_scan_config.sick_scan_res)/10000 << " (deg)" << std::endl;
    std::cout << "\tScan Area: " <<  "[" << ((double)_sick_scan_config.sick_start_angle)/10000 << "," << ((double)_sick_scan_config.sick_stop_angle)/10000 << "]" << std::endl;
    std::cout << std::endl;
    
  }
  
} //namespace SickToolbox
