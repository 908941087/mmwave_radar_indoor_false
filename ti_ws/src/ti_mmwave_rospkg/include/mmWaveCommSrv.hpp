
#ifndef MMWAVE_COMM_SRV_H
#define MMWAVE_COMM_SRV_H

/*Include ROS specific headers*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial/serial.h"  
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

/*Include standard C/C++ headers*/
#include <iostream>
#include <cstdio>
#include <sstream>

/*mmWave Driver Headers*/
#include <ti_mmwave_rospkg/mmWaveCLI.h>

namespace ti_mmwave_rospkg
{

class mmWaveCommSrv : public nodelet::Nodelet
{
   public:
   
   mmWaveCommSrv();
   
   private:
   
   virtual void onInit();
   
   bool commSrv_cb(ti_mmwave_rospkg::mmWaveCLI::Request  &req, ti_mmwave_rospkg::mmWaveCLI::Response &res);
   
   ros::ServiceServer commSrv;
   
   std::string mySerialPort;
   
   int myBaudRate;
}; //Class mmWaveCommSrv 

} //namespace ti_mmwave_rospkg 

#endif
