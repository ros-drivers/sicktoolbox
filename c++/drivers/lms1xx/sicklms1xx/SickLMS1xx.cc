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
    //SickLIDAR< SickLMS1xxBufferMonitor, SickLMS1xxMessage >( ),
    _sick_ip_address(sick_ip_address),
    _sick_tcp_port(sick_tcp_port)
  { }

  /**
   * A standard destructor
   */
  SickLMS1xx::~SickLMS1xx( ) { }

  /**
   * \brief Initializes the driver and syncs it with Sick LD unit. Uses sector config given in flash.
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
      std::cout << "\tAttempting to get device status..." << std::endl;
      //_syncDriverWithSick();
      _getSickStatus();
      
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
    //_printInitFooter();

    /* Success */
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
  void SickLMS1xx::_getSickStatus( ) throw( SickTimeoutException, SickIOException ) {

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

    /* Setup recv message */
    const uint8_t recv_message_header[10] = {0x02,
					      's',
				              'R',
				              'A',
				              ' ',
				              'S',
				              'T',
				              'l',
				              'm',
				              's'};
    
    /* Setup container for recv message */
    SickLMS1xxMessage recv_message;

    /* Send message and get reply using parent's method */
    try {
      
      SickLIDAR< SickLMS1xxBufferMonitor, SickLMS1xxMessage >::_sendMessageAndGetReply(send_message,
										       recv_message,
										       recv_message_header,
										       sizeof(recv_message_header),
										       0,
										       DEFAULT_SICK_LMS_1XX_MESSAGE_TIMEOUT,
										       1);

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

    //recv_message.Print();
    
    /* Success */

  }

} //namespace SickToolbox
