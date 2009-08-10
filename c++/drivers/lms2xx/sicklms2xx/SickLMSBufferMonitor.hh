/*!
 * \file SickLMSBufferMonitor.hh
 * \brief Defines a class for monitoring the receive
 *        buffer when interfacing w/ a Sick LMS LIDAR.
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

#ifndef SICK_LMS_BUFFER_MONITOR_HH
#define SICK_LMS_BUFFER_MONITOR_HH

#define DEFAULT_SICK_LMS_SICK_BYTE_TIMEOUT      (35000)  ///< Max allowable time between consecutive bytes

/* Definition dependencies */
#include "SickLMSMessage.hh"
#include "SickBufferMonitor.hh"
#include "SickException.hh"

/* Associate the namespace */
namespace SickToolbox {

  /*!
   * \brief A class for monitoring the receive buffer when interfacing with a Sick LMS LIDAR
   */
  class SickLMSBufferMonitor : public SickBufferMonitor< SickLMSBufferMonitor, SickLMSMessage > {

  public:

    /** A standard constructor */
    SickLMSBufferMonitor( );

    /** A method for extracting a single message from the stream */
    void GetNextMessageFromDataStream( SickLMSMessage &sick_message ) throw( SickIOException );

    /** A standard destructor */
    ~SickLMSBufferMonitor( );

  };
    
} /* namespace SickToolbox */

#endif /* SICK_LMS_BUFFER_MONITOR_HH */
