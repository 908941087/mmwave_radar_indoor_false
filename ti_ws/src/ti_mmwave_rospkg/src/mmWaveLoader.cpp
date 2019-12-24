/**
 * @file mmWaveLoader.cpp
 * @version 0.0.0
 * @author yzc
 * @brief 这个文档实现了通过ROS节点的功能启动mmWaveDataHdl和mmWaveCommSrv线程。
 */
#include "ros/ros.h"
#include "nodelet/loader.h"

/**
 * @brief main()函数是这个类的入口，它创建了名为mmWave_Manager的ROS节点，并通过nodelet的方式启动了mmWaveDataHdl和mmWaveCommSrv线程。
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "mmWave_Manager");
  
  nodelet::Loader manager(true);
  
  nodelet::M_string remap(ros::names::getRemappings());
  
  nodelet::V_string nargv;
  
  manager.load("mmWaveCommSrv", "ti_mmwave_rospkg/mmWaveCommSrv", remap, nargv);
  
  manager.load("mmWaveDataHdl", "ti_mmwave_rospkg/mmWaveDataHdl", remap, nargv);
  
  ros::spin();
  
  return 0;
}
