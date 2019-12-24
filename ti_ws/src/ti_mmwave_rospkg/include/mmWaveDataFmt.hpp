
#ifndef MMWAVE_DATA_FMT_H
#define MMWAVE_DATA_FMT_H

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

namespace ti_mmwave_rospkg
{

class mmWaveDataFmt : public nodelet::Nodelet
{
   public:
   
   mmWaveDataFmt();
   
   private:
   
   virtual void onInit();
   
   //laserscan publisher
   ros::Publisher ;
   
   //pcl publisher
   ros::Publisher ;
   
   //radar message subscriber
   ros::Subscriber ;
   
}; //Class mmWaveDataFmt 

} //namespace ti_mmwave_rospkg 

#endif
