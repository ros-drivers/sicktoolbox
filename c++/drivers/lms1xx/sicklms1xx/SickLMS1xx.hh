/*!
 * \file SickLMS1xx.hh
 * \brief Defines the SickLMS1xx class for working with the
 *        Sick LMS1xx laser range finders.
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

#ifndef SICK_LMS_1XX_HH
#define SICK_LMS_1XX_HH

/* Macros */
#define DEFAULT_SICK_LMS_1XX_IP_ADDRESS                   "192.168.0.1"  //< Default IP Address
#define DEFAULT_SICK_LMS_1XX_TCP_PORT                            (2111)  //< Sick LMS 1xx TCP/IP Port
#define DEFAULT_SICK_LMS_1XX_CONNECT_TIMEOUT                  (1000000)  //< Max time for establishing connection (usecs)
#define DEFAULT_SICK_LMS_1XX_MESSAGE_TIMEOUT                  (1000000)  //< Max time for reply (usecs)

#define SICK_LMS_1XX_MAX_BUFFER_LENGTH                           (2604)  //< Maximum number of bytes

/* Definition dependencies */
#include <string>
#include <arpa/inet.h>

#include "SickLIDAR.hh"
#include "SickLMS1xxBufferMonitor.hh"
#include "SickLMS1xxMessage.hh"
#include "SickException.hh"

/**
 * \namespace SickToolbox
 * \brief Encapsulates the Sick LIDAR Matlab/C++ toolbox
 */
namespace SickToolbox {

  /**
   * \class SickLMS1xx
   * \brief Provides a simple driver interface for working with the
   *        Sick LD-OEM/Sick LD-LRS long-range models via Ethernet.
   */
  class SickLMS1xx : public SickLIDAR< SickLMS1xxBufferMonitor, SickLMS1xxMessage > {

  public:

    /*!
     * \enum sick_lms_1xx_status_t 
     * \brief Defines the Sick LMS 1xx status.
     * This enum lists all of the Sick LMS 1xx status.
     */
    enum sick_lms_1xx_status_t {
      
      SICK_LMS_1XX_STATUS_UNDEFINED = 0x00,                                             ///< LMS 1xx status indefined
      SICK_LMS_1XX_STATUS_INITIALIZATION = 0x01,                                        ///< LMS 1xx initializing
      SICK_LMS_1XX_STATUS_CONFIGURATION = 0x02,                                         ///< LMS 1xx configuration
      SICK_LMS_1XX_STATUS_IDLE = 0x03,                                                  ///< LMS 1xx is idle
      SICK_LMS_1XX_STATUS_ROTATED = 0x04,                                               ///< LMS 1xx mirror rotating
      SICK_LMS_1XX_STATUS_IN_PREP = 0x05,                                               ///< LMS 1xx in preparation
      SICK_LMS_1XX_STATUS_READY = 0x06,                                                 ///< LMS 1xx is ready
      SICK_LMS_1XX_STATUS_READY_FOR_MEASUREMENT = 0x07                                  ///< LMS 1xx is ready to give measurements 

    };

    /*!
     * \struct sick_lms_1xx_scan_config_tag
     * \brief A structure for aggregrating the
     *        Sick LMS 1xx configuration params.
     */
    /*!
     * \typedef sick_lms_1xx_scan_config_t
     * \brief Adopt c-style convention
     */
    typedef struct sick_lms_1xx_scan_config_tag {
      uint32_t sick_scan_freq;                                                          ///< Sick system software version
      uint32_t sick_scan_res;                                                           ///< Sick system software version      
      int32_t sick_start_angle;                                                         ///< Sick boot prom software version
      int32_t sick_stop_angle;                                                          ///< Sick boot prom software version
    } sick_lms_1xx_scan_config_t;
    
    /** Primary constructor */
    SickLMS1xx( const std::string sick_ip_address = DEFAULT_SICK_LMS_1XX_IP_ADDRESS,
		const uint16_t sick_tcp_port = DEFAULT_SICK_LMS_1XX_TCP_PORT );
    
    /** Initializes the Sick LD unit (use scan areas defined in flash) */
    void Initialize( )  throw( SickIOException, SickThreadException, SickTimeoutException, SickErrorException );

    /** Sets the scan configuration (volatile, does not write to EEPROM) */
    void SetSickScanConfig( ) throw( SickTimeoutException, SickIOException, SickConfigException );
    
    /** Uninitializes the Sick LD unit */
    void Uninitialize( ) throw( SickIOException, SickTimeoutException, SickErrorException, SickThreadException );

    /** Destructor */
    ~SickLMS1xx();

  private:

    /** The Sick LMS 1xx IP address */
    std::string _sick_ip_address;

    /** The Sick LMS 1xx TCP port number */
    uint16_t _sick_tcp_port;

    /** Sick LMS 1xx socket address struct */
    struct sockaddr_in _sick_inet_address_info;

    /** Sick LMS 1xx configuration struct */
    sick_lms_1xx_scan_config_t _sick_scan_config;
    
    /** Setup the connection parameters and establish TCP connection! */
    void _setupConnection( ) throw( SickIOException, SickTimeoutException );  

    /** Teardown the connection to the Sick LD */
    void _teardownConnection( ) throw( SickIOException );

    /** Acquire the Sick LMS's status */
    void _getSickStatus( sick_lms_1xx_status_t &sick_status, bool &temp_status ) throw( SickTimeoutException, SickIOException );

    /** Acquire the Sick LMS's scan config */
    void _getSickScanConfig( ) throw( SickTimeoutException, SickIOException );

    /** Set access mode for configuring device */
    bool _setAuthorizedClientAccessMode() throw( SickTimeoutException, SickIOException );
    
    /** Send the message and grab expected reply */
    void _sendMessageAndGetReply( const SickLMS1xxMessage &send_message,
				  SickLMS1xxMessage &recv_message,
				  const std::string reply_command_code,
				  const std::string reply_command,
				  const unsigned int timeout_value = DEFAULT_SICK_LMS_1XX_MESSAGE_TIMEOUT,
				  const unsigned int num_tries = 1 ) throw( SickIOException, SickTimeoutException );

    /** Utility function to convert int to status */
    sick_lms_1xx_status_t _intToSickStatus( const int status ) const;

    /** Utility function to convert config error int to str */
    std::string _intToSickConfigErrorStr( const int error ) const;

    /** Utility function for printing Sick scan config */
    void _printSickScanConfig( ) const;
    
    /** Utility function for printing footer after initialization */
    void _printInitFooter( ) const;
    
  };

  /*!
   * \typedef sick_lms_1xx_type_t
   * \brief Makes working w/ SickLMS1xx::sick_lms_1xx_status_t a bit easier
   */
  typedef SickLMS1xx::sick_lms_1xx_status_t sick_lms_1xx_status_t;
  
} //namespace SickToolbox
  
#endif /* SICK_LMS_1XX_HH */
