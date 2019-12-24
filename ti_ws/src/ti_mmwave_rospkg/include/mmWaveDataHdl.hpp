
#ifndef MMWAVE_DATA_HDL_H
#define MMWAVE_DATA_HDL_H

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
#include "DataHandlerClass.h"

namespace ti_mmwave_rospkg
{

class mmWaveDataHdl : public nodelet::Nodelet
{  
   public:
   
   mmWaveDataHdl();
   
   private:
   
   virtual void onInit();
   
   //char* mySerialPort;
   
   //int myBaudRate;
   
}; //Class mmWaveDataHdl 

} //namespace ti_mmwave_rospkg 

#endif
