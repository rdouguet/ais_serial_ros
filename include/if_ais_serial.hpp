/*
 * Copyright © 2022 , Universite Bretagne Sud, Lab-STICC, Ronan Douguet & Dominique Heller
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the “Software”), to deal in the Software
 * without restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies
 * or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
 * OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef IF_AIS_SERIAL_HPP
#define IF_AIS_SERIAL_HPP

/* includes ----------------------------------------------------------------*/
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include <marnav/nmea/nmea.hpp>
#include <marnav/nmea/vdm.hpp>
#include <marnav/nmea/ais_helper.hpp>
#include <marnav/ais/ais.hpp>
#include <marnav/ais/message_01.hpp>
#include <marnav/ais/message_02.hpp>
#include <marnav/ais/message_03.hpp>
#include <marnav/ais/message_05.hpp>
#include <marnav/ais/message_18.hpp>
#include <marnav/ais/message_24.hpp>
#include <marnav/io/default_nmea_reader.hpp>
#include <marnav/utils/unique.hpp>
#include <boost/asio.hpp>

#include <ais_serial_ros/ais_data.h>

/* defines -----------------------------------------------------------------*/
/* strutures ---------------------------------------------------------------*/
/* constant ----------------------------------------------------------------*/
/* global variables --------------------------------------------------------*/
/* class -------------------------------------------------------------------*/
class ifAisSerial
{
public:

  // paramAis contains parameters provided by the launch file
  struct paramAis
  {
    // USB port com information
	  std::string port_name;
	  int baud_rate;

    // network parameters
    std::string addr_udp_server;
    int udp_port;
    bool udp_output_enable;

    // display or not the debug information
    bool debug_info = false;
  };

  /*==============================================================*/
  /* Constructor                                                  */
  /*==============================================================*/

  /**
    @fn ifAisSerial(ros::NodeHandle& ref_node_handle)
      Default Constructor
    @param ref_node_handle is a pointer on the ros NodeHandle
  */
  ifAisSerial(ros::NodeHandle& ref_node_handle);

  /**
    @fn ifAisSerial(ros::NodeHandle& ref_node_handle)
      Default Destructor
  */
  ~ifAisSerial();

  /*==============================================================*/
  /* Pulic methods                                                */
  /*==============================================================*/

  /**
    @fn bool initUdpNetwork()
      Initialize the udp networtk to transmit ais udp frame
    @return true if the initialization is ok (else false)
  */
  bool initUdpNetwork();

  /**
    @fn bool readAisFrame()
     Read the ais frame
    @return true if the ais frame is received (else false)
  */
  bool readAisFrame();

  /**
    @fn bool closeUdpNetwork()
      Close the udp network
    @return true if the udp socket is correctly close (else false)
  */
  bool closeUdpNetwork();

  /**
    @fn bool initSubAndPubMsg()
      Initialize the subscribed and published messages
    @return true if the init is ok (else false)
  */
  bool initSubAndPubMsg();

private:

  /*==============================================================*/
  /* Private variables                                            */
  /*==============================================================*/

  ros::NodeHandle&     m_nh_;
  ros::Publisher       m_pub_raw_data_;
  ros::Publisher       m_pub_ais_data_;

  paramAis             m_param_ais_;

  // vector of the nmea sentence object
  std::vector<std::unique_ptr<marnav::nmea::sentence>> m_vector_sentences_;
	std::string m_data_frame_;
	int m_last_fragment_index_ = 0;

 	char *m_tx_buffer_;
	struct sockaddr_in m_servaddr_ = {0};
	int m_sockfd_;
  socklen_t m_len_ = 0;

  /*==============================================================*/
  /* Private methods                                              */
  /*==============================================================*/

  /**
    @fn void loadParameters(void)
      Load parameters provided by the launch file
  */
  void loadParameters(void);

};


#endif // IF_AIS_SERIAL_HPP

