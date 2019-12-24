/** 
 * @file mmWaveQuickConfig.cpp
 * @version 0.0.0
 * @author yzc
 * @brief 这个文件加载了txt文件并做了转换，将每行的数据发送到mmWaveCLI服务中，并从服务中获得回复的消息。
 */

#include "ros/ros.h"
#include "ti_mmwave_rospkg/mmWaveCLI.h"
#include <cstdlib>
#include <fstream>
#include <stdio.h>
#include <regex>

/**
 * @brief 创建mmWaveQuickConfig线程，并创建mmWaveCLI服务，读取了雷达的配置文件并通过ROS服务机制向服务发送配置文件的信息，调用相应的服务函数将配置信息写入
 * 雷达中，并获得服务函数的回复。首先，函数等待服务函数的创建。然后，函数从配置文件中获得每行的数据，如果数据是空白的一行或者回车键键入的信息，函数会忽略，不会将
 * 信息传入服务函数中。如果是数据信息则传入函数中等待服务函数处理完毕，回复请求。最后，将文件数据全部读入以后关闭文件。
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mmWaveQuickConfig");
  if (argc != 2)
  {
    ROS_INFO("mmWaveQuickConfig: usage: mmWaveQuickConfig /file_directory/params.cfg");
    return 1;
  }
  else
  {
    ROS_INFO("mmWaveQuickConfig: Configuring mmWave device using config file: %s", argv[1]);
  }
  
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<ti_mmwave_rospkg::mmWaveCLI>("/mmWaveCommSrv/mmWaveCLI");
  ti_mmwave_rospkg::mmWaveCLI srv;
  std::ifstream myParams;
  
  /// 等待服务创建完毕
  ros::service::waitForService("/mmWaveCommSrv/mmWaveCLI", 100000); 
  
  myParams.open(argv[1]);
  
  if( myParams.is_open() )
  {
    while( std::getline(myParams, srv.request.comm) )
    {
      srv.request.comm.erase(std::remove(srv.request.comm.begin(), srv.request.comm.end(), '\r'), srv.request.comm.end());

      /// 忽略换行以及空白行
      if (std::regex_match(srv.request.comm, std::regex("^\\s*%.*") ) ||
          std::regex_match(srv.request.comm, std::regex("^\\s*") ))
      {
          ROS_INFO("mmWaveQuickConfig: Ignored blank or comment line: '%s'", srv.request.comm.c_str() );
      }

      /// 向服务发送配置文件的信息
      else
      {
        ROS_INFO("mmWaveQuickConfig: Sending command: '%s'", srv.request.comm.c_str() );
	int numTries = 0;
        while (numTries < 2)
	{
          if( client.call(srv) )
          {
            if (std::regex_search(srv.response.resp, std::regex("Done") )) 
            {
              ROS_INFO("mmWaveQuickConfig: Command successful (mmWave sensor responded with 'Done')");
	      break;
            }
            else if (numTries == 0)
            {
              ROS_INFO("mmWaveQuickConfig: Command failed (mmWave sensor did not respond with 'Done')");
              ROS_INFO("mmWaveQuickConfig: Response: '%s'", srv.response.resp.c_str() );
            }
	    else
	    {
              ROS_ERROR("mmWaveQuickConfig: Command failed (mmWave sensor did not respond with 'Done')");
              ROS_ERROR("mmWaveQuickConfig: Response: '%s'", srv.response.resp.c_str() );
              return 1;
	    }
          }
          else
          {
            ROS_ERROR("mmWaveQuickConfig: Failed to call service mmWaveCLI");
            ROS_ERROR("%s", srv.request.comm.c_str() );
            return 1;
          }
	  numTries++;
	}
      }
    }
    /// 关闭文件
    myParams.close();
  }
  else
  {
     ROS_ERROR("mmWaveQuickConfig: Failed to open File %s", argv[1]);
     return 1;
  }

  ROS_INFO("mmWaveQuickConfig: mmWaveQuickConfig will now terminate. Done configuring mmWave device using config file: %s", argv[1]);
  return 0;
}
