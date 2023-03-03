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

/* includes -----------------------------------------------------------------*/
#include "if_ais_serial.hpp"
#include "if_serial_boost.hpp"

/* namespace ----------------------------------------------------------------*/
using namespace marnav;
using namespace nmea;

/* defines ------------------------------------------------------------------*/
/* public methods -----------------------------------------------------------*/

//=============================================================================
/**
  @fn ifAisSerial::ifAisSerial()
    Class Constructor
*/
ifAisSerial::ifAisSerial(ros::NodeHandle& ref_node_handle):
m_nh_(ref_node_handle)
{
  this->loadParameters();
}

//=============================================================================
/**
  @fn ifAisSerial::~ifAisSerial()
    Class Destructor
*/
ifAisSerial::~ifAisSerial(void)
{
}

//=============================================================================
/**
  @fn bool initUdpNetwork()
    Initialize the udp networtk to receiver radar frame
  @return true if the initialization is ok (else false)
*/
bool ifAisSerial::initUdpNetwork()
{
  ROS_INFO("Init udp network changement");

  // initialize the socket ----------------------------
  this->m_sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);

  if(this->m_sockfd_ == -1)
	{
		ROS_INFO("failed to create socket");
		return false;
	}

  // set address and port -----------------------------
  this->m_servaddr_.sin_family = AF_INET;
  this->m_servaddr_.sin_port = htons(this->m_param_ais_.udp_port);
  this->m_servaddr_.sin_addr.s_addr = inet_addr(this->m_param_ais_.addr_udp_server.c_str()); // INADDR_ANY; //inet_addr(this->m_param_radar_.addr_udp_server.c_str());

  if(this->m_param_ais_.debug_info)
  {
    ROS_INFO("udp port after htons function: %d", this->m_servaddr_.sin_port);
  }

  return true;
}

//=============================================================================
/**
  @fn bool readAisFrame()
    Read the ais frame
  @return true if the ais frame is received (else false)
*/
bool ifAisSerial::readAisFrame()
{
	static marnav::io::default_nmea_reader input{marnav::utils::make_unique<IfSerialBoost>(this->m_param_ais_.port_name, this->m_param_ais_.baud_rate)};

	bool ret_val = false;
  	ais_serial_ros::ais_data msg_ais; // message for ais data

  	try
  	{
		if(input.read_sentence(this->m_data_frame_))
		{
	  		// save the received data ===============================
	  		std_msgs::String str_raw_ais_frame;
	  		str_raw_ais_frame.data = this->m_data_frame_;

	  		if(this->m_param_ais_.debug_info)
			{
				ROS_INFO_STREAM(str_raw_ais_frame.data);
			}

			// Parse all received frames ============================
			try
			{
				auto s = nmea::make_sentence(str_raw_ais_frame.data);  // s is the nmea sentence objet (check the validation of nmea frame)
				auto s1 = nmea::make_sentence(str_raw_ais_frame.data); // need to use the second objet sentance since sentence_cast and std_move change this object

				if (s->id() == nmea::sentence_id::VDM) // check if it is a AIS frame (AIVDM)
				{
					// parse the frame to get the number of fragments for the AIS frame
					auto vdm = nmea::sentence_cast<nmea::vdm>(s1);
					auto nb_fragments = vdm->get_n_fragments();
					auto fragment = vdm->get_fragment();
					ROS_DEBUG_STREAM("fragment "<<fragment<<" on "<<nb_fragments);

					// check if the fragment index is correctly incremented
					if((this->m_last_fragment_index_ + 1) != fragment)
					{
						// erase the vector sentences
						this->m_vector_sentences_.erase (this->m_vector_sentences_.begin(),this->m_vector_sentences_.end());
					}

					this->m_vector_sentences_.push_back(std::move(s)); // add this new sentence on the vector

					// published the ais raw frame
					this->m_pub_raw_data_.publish(str_raw_ais_frame);

					if(this->m_param_ais_.udp_output_enable)
					{
						this->m_tx_buffer_ = &(str_raw_ais_frame.data[0]);

						int len = sendto(m_sockfd_, (const char *)m_tx_buffer_, strlen(m_tx_buffer_), 0, (const struct sockaddr *)&m_servaddr_, sizeof(m_servaddr_));

  						if(len == -1)
  						{
							ROS_INFO("failed to send");
  						}

					}

					// check if the ais frame is complete
					if (fragment == nb_fragments)
					{
						// parse and process AIS messags
						auto payload = nmea::collect_payload(this->m_vector_sentences_.begin(), this->m_vector_sentences_.end());
						auto message = ais::make_message(payload);

						// display id of message
						if(this->m_param_ais_.debug_info)
						{
							ROS_INFO("Message id: %d", (int)message->type());
						}

						switch (message->type())
						{
							// MESSAGE_ID_01 ------------------------
							case ais::message_id::position_report_class_a:
							{
								// parse the ais field for message 01
								auto report = ais::message_cast<ais::message_01>(message);

								// fill some parameters of ais message
								msg_ais.mmsi = report->get_mmsi();
								msg_ais.nav_status = (uint8_t)report->get_nav_status();
								msg_ais.longitude = report->get_lon().value();
								msg_ais.latitude = report->get_lat().value();
								msg_ais.cog = report->get_cog().value();
								msg_ais.sog = report->get_sog().value().value(); // TODO check the value....
								msg_ais.hdg = report->get_hdg().value();
								//msg_ais.rot = report->get_rot();	// TODO probleme with the value type
							}
							break;

							// MESSAGE_ID_02 ------------------------
							case ais::message_id::position_report_class_a_assigned_schedule:
							{
								// parse the ais field for message 02
								auto report = ais::message_cast<ais::message_02>(message);

								// fill some parameters of ais message
								msg_ais.mmsi = report->get_mmsi();
								msg_ais.nav_status = (uint8_t)report->get_nav_status();
								msg_ais.longitude = report->get_lon().value();
								msg_ais.latitude = report->get_lat().value();
								msg_ais.cog = report->get_cog().value();
								msg_ais.sog = report->get_sog().value().value(); // TODO check the value....
								msg_ais.hdg = report->get_hdg().value();
								//msg_ais.rot = report->get_rot();	// TODO probleme with the value type
							}
							break;

							// MESSAGE_ID_03 ------------------------
							case ais::message_id::position_report_class_a_response_to_interrogation:
							{
								// parse the ais field for message 03
								auto report = ais::message_cast<ais::message_03>(message);

								// fill some parameters of ais message
								msg_ais.mmsi = report->get_mmsi();
								msg_ais.nav_status = (uint8_t)report->get_nav_status();
								msg_ais.longitude = report->get_lon().value();
								msg_ais.latitude = report->get_lat().value();
								msg_ais.cog = report->get_cog().value();
								msg_ais.sog = report->get_sog().value().value(); // TODO check the value....
								msg_ais.hdg = report->get_hdg().value();
								//msg_ais.rot = report->get_rot();	// TODO probleme with the value type
							}
							break;

							// MESSAGE_ID_05 ------------------------
							case ais::message_id::static_and_voyage_related_data:
							{
								// parse the ais field for message 05
								auto report = ais::message_cast<ais::message_05>(message);

								// fill some parameters of ais message
								msg_ais.mmsi = report->get_mmsi();
								msg_ais.shipname.data = report->get_shipname();
								msg_ais.imo_number = report->get_imo_number();
								msg_ais.ship_type = (uint8_t)report->get_shiptype();
							}
							break;

							// MESSAGE_ID_18 ------------------------
							case ais::message_id::standard_class_b_cs_position_report:
							{
								// parse the ais field for message 18
								auto report = ais::message_cast<ais::message_18>(message);

								// fill some parameters of ais message
								msg_ais.mmsi = report->get_mmsi();
								msg_ais.longitude = report->get_lon().value();
								msg_ais.latitude = report->get_lat().value();
								msg_ais.cog = report->get_cog().value();
								msg_ais.sog = report->get_sog().value().value(); // TODO check the value....
								msg_ais.hdg = report->get_hdg().value();
							}
							break;

							// MESSAGE_ID_24 ---------------------------
							case ais::message_id::static_data_report:
							{
								// parse the ais field for message 24
								auto report = ais::message_cast<ais::message_24>(message);

								// fill some parameters of ais message
								msg_ais.mmsi = report->get_mmsi();
								msg_ais.shipname.data = report->get_shipname();
								msg_ais.ship_type = (uint8_t)report->get_shiptype();
							}
							break;
						}

						// erase the vector sentences
						this->m_vector_sentences_.erase (this->m_vector_sentences_.begin(),this->m_vector_sentences_.end());

						// published the ais message
						this->m_pub_ais_data_.publish(msg_ais);

						// reset lhe last fragment index
						this->m_last_fragment_index_ = 0;
					}
					else
					{
						// save the last fragment
						this->m_last_fragment_index_ = fragment;
					}
				}
			}
			catch(std::exception& ex)
			{
				this->m_vector_sentences_.erase (m_vector_sentences_.begin(),m_vector_sentences_.end());
				ROS_INFO("bad ais frame..."); // TODO -> get the error (checksum_error, unknown_sentence...)
			}
		}
	}
	catch(std::exception& ex)
	{
		ROS_INFO_STREAM("error:"<<ex.what()); // TODO -> get the error (checksum_error, unknown_sentence...)
	}

	return ret_val;
}

//=============================================================================
/**
  @fn bool closeUdpNetwork()
    Close the udp network
  @return true if the udp socket is correctly close (else false)
*/
bool ifAisSerial::closeUdpNetwork()
{
	close(this->m_sockfd_);

  return true;
}

//=============================================================================
/**
  @fn bool initSubAndPubMsg()
    Initialize the subscribed and published messages
  @return true if the init is ok (else false)
*/
bool ifAisSerial::initSubAndPubMsg()
{
  this->m_pub_raw_data_ = this->m_nh_.advertise<std_msgs::String>("ais_serial/raw_frame", 10);  // 10 => queue_size
  this->m_pub_ais_data_ = this->m_nh_.advertise<ais_serial_ros::ais_data>("ais_serial/data", 10);  // 10 => queue_size

  return true;
}

/* private methods ----------------------------------------------------------*/

//=============================================================================
  /**
    @fn void loadParameters(void)
      Load parameters provided by the launch file
  */
void ifAisSerial::loadParameters(void)
{
  // get parameters provided by the yaml config file ---------------
  this->m_nh_.getParam("/ais_serial_ros_node/portName", this->m_param_ais_.port_name);
  this->m_nh_.getParam("/ais_serial_ros_node/baudRate", this->m_param_ais_.baud_rate);

  // get parameters from launch file -------------------------------
  this->m_nh_.getParam("/ais_serial_ros_node/addr_udp_server", this->m_param_ais_.addr_udp_server);
  this->m_nh_.getParam("/ais_serial_ros_node/udp_port", this->m_param_ais_.udp_port);
  this->m_nh_.getParam("/ais_serial_ros_node/udp_output_enable", this->m_param_ais_.udp_output_enable);

  this->m_nh_.getParam("/ais_serial_ros_node/debug_info", this->m_param_ais_.debug_info);

  // display some parameters for check -----------------------------
  if(this->m_param_ais_.debug_info)
  {
    ROS_INFO("PARAMETERS =========================");
	ROS_INFO_STREAM("portName: " << this->m_param_ais_.port_name);
	ROS_INFO_STREAM("baudrate: " << this->m_param_ais_.baud_rate);
    ROS_INFO("address udp server : %s", this->m_param_ais_.addr_udp_server.c_str());
    ROS_INFO("udp port: %d", this->m_param_ais_.udp_port);
  }
}
