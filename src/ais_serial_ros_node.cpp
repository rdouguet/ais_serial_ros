
/*
 * Copyright © 2022 , Universite Bretagne Sud, Lab-STICC, Ronan Douguet
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

/* includes ----------------------------------------------------------------*/
#include <if_ais_serial.hpp>

/* macro -------------------------------------------------------------------*/
/* defines -----------------------------------------------------------------*/
/* strutures ---------------------------------------------------------------*/
/* constant ----------------------------------------------------------------*/
/* global variables --------------------------------------------------------*/

//============================================================================
/**
    @fn int main(int argc, char** argv)
        main code
    @return
*/
int main (int argc, char** argv)
{
  //using namespace marnav;

  // node initialisation -------------------------------------------
  ros::init(argc, argv, "ais_serial_ros_node");
  ros::NodeHandle node_handle;

  // instantiate the ais interface ---------------------------------
  ifAisSerial if_ais(node_handle);

  // initialize this interface -------------------------------------
  if(if_ais.initUdpNetwork() == false)
  {
   	ROS_INFO_STREAM(" error initialization");
    return 0;
  }

  if(if_ais.initSubAndPubMsg() == false)
  {
    ROS_INFO_STREAM(" error init pub message");
    return 0;
  }

  // Read and parse the ais data every 10 Hz -----------------------
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    // check if ais frame have been received
	  if_ais.readAisFrame();

    ros::spinOnce();
    loop_rate.sleep();
  }

  // close socket --------------------------------------------------
  if(if_ais.closeUdpNetwork() == false)
  {
    ROS_INFO_STREAM(" error close socket");
  }

  return 0;
}
