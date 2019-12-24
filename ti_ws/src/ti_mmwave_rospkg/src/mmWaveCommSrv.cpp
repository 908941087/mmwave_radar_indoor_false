/**
 * @file mmWaveCommSrv.cpp
 * @version 0.0.0
 * @author yzc
 * @brief 这个文件通过ROS的节点线程打开了一个特定波特率的用户串行端口，这个端口被用来向雷达板传输配置数据。
 *                     
 */
#include "mmWaveCommSrv.hpp"

namespace ti_mmwave_rospkg
{

PLUGINLIB_EXPORT_CLASS(ti_mmwave_rospkg::mmWaveCommSrv, nodelet::Nodelet);

mmWaveCommSrv::mmWaveCommSrv() {}

/**
 * @brief 初始化的函数设置了端口的名称以及数据传输的波特率。之后创建了一项名为mmWaveCLI的服务。当服务器请求服务函数的时候，函数就会打开雷达的写入端口，并将服务器传输过来的
 * 数据写入雷达中。
 */ 
void mmWaveCommSrv::onInit()
{
   ros::NodeHandle private_nh = getPrivateNodeHandle();
   
   private_nh.getParam("/mmWave_Manager/command_port", mySerialPort);
   
   private_nh.getParam("/mmWave_Manager/command_rate", myBaudRate);
   
   ROS_INFO("mmWaveCommSrv: command_port = %s", mySerialPort.c_str());
   ROS_INFO("mmWaveCommSrv: command_rate = %d", myBaudRate);
   
   commSrv = private_nh.advertiseService("mmWaveCLI", &mmWaveCommSrv::commSrv_cb, this);
   
   NODELET_DEBUG("mmWaveCommsrv: Finished onInit function");
}

/**
 * @brief 该函数是当其他类请求mmWaveCLI服务时，就会调用此函数。此函数会打开雷达的写端口，并将服务器请求的内容写入雷达中，并返回雷达回复的内容
 */
bool mmWaveCommSrv::commSrv_cb(mmWaveCLI::Request &req , mmWaveCLI::Response &res)
{
   NODELET_DEBUG("mmWaveCommSrv: Port is \"%s\" and baud rate is %d", mySerialPort.c_str(), myBaudRate);

   /// 打开端口并进行错误检测
   serial::Serial mySerialObject("", myBaudRate, serial::Timeout::simpleTimeout(1000));
   mySerialObject.setPort(mySerialPort.c_str());
   try
   {
      mySerialObject.open();
   } catch (std::exception &e1) {
      ROS_INFO("mmWaveCommSrv: Failed to open User serial port with error: %s", e1.what());
      ROS_INFO("mmWaveCommSrv: Waiting 20 seconds before trying again...");
      try
      {
         ros::Duration(20).sleep();
         mySerialObject.open();
      } catch (std::exception &e2) {
         ROS_ERROR("mmWaveCommSrv: Failed second time to open User serial port, error: %s", e1.what());
         NODELET_ERROR("mmWaveCommSrv: Port could not be opened. Port is \"%s\" and baud rate is %d", mySerialPort.c_str(), myBaudRate);
	 return false;
      }
   }

   /// 读取之前等待的回复
   while (mySerialObject.available() > 0)
   {
      mySerialObject.readline(res.resp, 1024, ":/>");
      ROS_INFO("mmWaveCommSrv: Received (previous) response from sensor: '%s'", res.resp.c_str());
      res.resp = "";
   }

   /// 向雷达写入数据
   ROS_INFO("mmWaveCommSrv: Sending command to sensor: '%s'", req.comm.c_str());
   req.comm.append("\n");
   int bytesSent = mySerialObject.write(req.comm.c_str());

   /// 从雷达读出数据
   mySerialObject.readline(res.resp, 1024, ":/>");
   ROS_INFO("mmWaveCommSrv: Received response from sensor: '%s'", res.resp.c_str());

   mySerialObject.close();
   
   return true;
}

}



