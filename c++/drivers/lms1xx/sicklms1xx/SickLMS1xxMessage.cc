/*!
 * \file SickLMS1xxMessage.cc
 * \brief Implements the class SickLMS1xxMessage.
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
#include <iomanip>
#include <iostream>
#include <arpa/inet.h> 

#include "SickLMS1xxMessage.hh"
#include "SickLMS1xxUtility.hh" // for byye-order conversions where necessary

/* Associate the namespace */
namespace SickToolbox {

  /**
   * \brief A default constructor
   */
  SickLMS1xxMessage::SickLMS1xxMessage( ) :
    SickMessage< SICK_LMS_1XX_MSG_HEADER_LEN, SICK_LMS_1XX_MSG_PAYLOAD_MAX_LEN, SICK_LMS_1XX_MSG_TRAILER_LEN >()  {

    /* Initialize the object */
    Clear(); 

  }
  
  /**
   * \brief Another constructor.
   * \param *payload_buffer The payload for the packet as an array of bytes (including the header)
   * \param payload_length The length of the payload array in bytes
   */
  SickLMS1xxMessage::SickLMS1xxMessage( const uint8_t * const payload_buffer, const unsigned int payload_length ) :
    SickMessage< SICK_LMS_1XX_MSG_HEADER_LEN, SICK_LMS_1XX_MSG_PAYLOAD_MAX_LEN, SICK_LMS_1XX_MSG_TRAILER_LEN >()  {

    /* Build the message object (implicit initialization) */
    BuildMessage(payload_buffer,payload_length); 

  }
  
  /**
   * \brief Another constructor.
   * \param *message_buffer A well-formed message to be parsed into the class' fields
   */
  SickLMS1xxMessage::SickLMS1xxMessage( const uint8_t * const message_buffer ) :
    SickMessage< SICK_LMS_1XX_MSG_HEADER_LEN, SICK_LMS_1XX_MSG_PAYLOAD_MAX_LEN, SICK_LMS_1XX_MSG_TRAILER_LEN >()  {

    /* Parse the message into the container (implicit initialization) */
    ParseMessage(message_buffer); 

  }
  
  /**
   * \brief Constructs a well-formed Sick LMS 1xx message
   * \param *payload_buffer An address of the first byte to be copied into the message's payload
   * \param payload_length The number of bytes to be copied into the message buffer
   */
  void SickLMS1xxMessage::BuildMessage( const uint8_t * const payload_buffer, const unsigned int payload_length ) {

    /* Call the parent method
     * NOTE: The parent method resets the object and assigns _message_length, _payload_length,
     *       _populated and copies the payload into the message buffer at the correct position
     */
    SickMessage< SICK_LMS_1XX_MSG_HEADER_LEN, SICK_LMS_1XX_MSG_PAYLOAD_MAX_LEN, SICK_LMS_1XX_MSG_TRAILER_LEN >
      ::BuildMessage(payload_buffer,payload_length);
    
    /*
     * Set the message header!
     */
    _message_buffer[0] = 0x02;                 // STX

    /*
     * Set the message trailer! 
     */
    _message_buffer[_message_length-1] = 0x03; // ETX

  }
  
  /**
   * \brief Parses a sequence of bytes into a SickLMS1xxMessage object
   * \param *message_buffer A well-formed message to be parsed into the class' fields
   */
  void SickLMS1xxMessage::ParseMessage( const uint8_t * const message_buffer ) {
    
    /* Call the parent method
     * NOTE: This method resets the object and assigns _populated as true
     */
    SickMessage< SICK_LMS_1XX_MSG_HEADER_LEN, SICK_LMS_1XX_MSG_PAYLOAD_MAX_LEN, SICK_LMS_1XX_MSG_TRAILER_LEN >
      ::ParseMessage(message_buffer);
    
    /* Compute the message length */
    //int i = 1;
    //while (message_buffer[i-1] != 0x03 || i > ) {

      /* Grab the command type */
    //  _command_type = strtok(_message_buffer," ");

    //    if ((token = strtok(&msg_buffer[pos]," ")) == NULL) {
    //std::cerr << "SickLMS1xx::GetSickMeasurements: strtok() failed!" << std::endl;
    //return false;
    //}
 
      /* Grab the command code */
      
    //  i++;
      
    }

    /* Compute the total message length */
    //_payload_length = _message_length - MESSAGE_HEADER_LENGTH - MESSAGE_TRAILER_LENGTH;
    
    /* Copy the given packet into the buffer */
    //memcpy(_message_buffer,message_buffer,_message_length);

  }

  /**
   * \brief Print the message contents.
   */
  void SickLMS1xxMessage::Print( ) const {

    //std::cout.setf(std::ios::hex,std::ios::basefield);
    //std::cout << "Checksum: " << (unsigned int) GetChecksum() << std::endl;  
    //std::cout << "Service code: " << (unsigned int) GetServiceCode() << std::endl;
    //std::cout << "Service subcode: " << (unsigned int) GetServiceSubcode() << std::endl;
    //std::cout << std::flush;

    /* Call parent's print function */
    SickMessage< SICK_LMS_1XX_MSG_HEADER_LEN, SICK_LMS_1XX_MSG_PAYLOAD_MAX_LEN, SICK_LMS_1XX_MSG_TRAILER_LEN >::Print();    

  }
  
  /**
   * \brief Compute the message checksum (single-byte XOR).
   * \param data The address of the first data element in a sequence of bytes to be included in the sum
   * \param length The number of byte in the data sequence
   */
  uint8_t SickLMS1xxMessage::_computeXOR( const uint8_t * const data, const uint32_t length ) {
    
    /* Compute the XOR by summing all of the bytes */
    uint8_t checksum = 0;
    //for (uint32_t i = 0; i < length; i++) {
    //  checksum ^= data[i]; // NOTE: this is equivalent to simply summing all of the bytes
    //}
    
    /* done */
    return checksum;

  }
  
  SickLMS1xxMessage::~SickLMS1xxMessage( ) { }
  
} /* namespace SickToolbox */
