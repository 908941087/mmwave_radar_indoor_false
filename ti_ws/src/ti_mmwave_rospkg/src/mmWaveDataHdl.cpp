/**
 * @file mmWaveDataHdl.cpp
 * @version 0.0.0
 * @author yzc
 * @brief 这个文件通过实现ROS的线程节点打开了雷达板的读数据串口
 */
#include "mmWaveDataHdl.hpp"
#include "DataHandlerClass.h"

namespace ti_mmwave_rospkg
{

PLUGINLIB_EXPORT_CLASS(ti_mmwave_rospkg::mmWaveDataHdl, nodelet::Nodelet);

mmWaveDataHdl::mmWaveDataHdl() {}

/**
 * @brief 这个函数通过节点句柄获取了雷达串口的配置参数，雷达所使用的UART串口通讯协议。这个函数实例化了DataUARTHandler类，并设置了串口的名称、波特率、雷达检测的最大水平
 * 角度与垂直角度。之后打开雷达串口获得雷达数据。
 */
void mmWaveDataHdl::onInit()
{
   ros::NodeHandle private_nh = getPrivateNodeHandle();
   
   std::string mySerialPort;
   int myBaudRate;
   int myMaxAllowedElevationAngleDeg;
   int myMaxAllowedAzimuthAngleDeg;
   
   private_nh.getParam("/mmWave_Manager/data_port", mySerialPort);
   
   private_nh.getParam("/mmWave_Manager/data_rate", myBaudRate);
   
   if (!(private_nh.getParam("/mmWave_Manager/max_allowed_elevation_angle_deg", myMaxAllowedElevationAngleDeg)))
   {
      myMaxAllowedElevationAngleDeg = 90;  // Use max angle if none specified
   }

   if (!(private_nh.getParam("/mmWave_Manager/max_allowed_azimuth_angle_deg", myMaxAllowedAzimuthAngleDeg)))
   {
      myMaxAllowedAzimuthAngleDeg = 90;  // Use max angle if none specified
   }

   ROS_INFO("mmWaveDataHdl: data_port = %s", mySerialPort.c_str());
   ROS_INFO("mmWaveDataHdl: data_rate = %d", myBaudRate);
   ROS_INFO("mmWaveDataHdl: max_allowed_elevation_angle_deg = %d", myMaxAllowedElevationAngleDeg);
   ROS_INFO("mmWaveDataHdl: max_allowed_azimuth_angle_deg = %d", myMaxAllowedAzimuthAngleDeg);
   
   DataUARTHandler DataHandler(&private_nh);
   DataHandler.setUARTPort( (char*) mySerialPort.c_str() );
   DataHandler.setBaudRate( myBaudRate );
   DataHandler.setMaxAllowedElevationAngleDeg( myMaxAllowedElevationAngleDeg );
   DataHandler.setMaxAllowedAzimuthAngleDeg( myMaxAllowedAzimuthAngleDeg );
   DataHandler.start();
   
   NODELET_DEBUG("mmWaveDataHdl: Finished onInit function");
}

}



