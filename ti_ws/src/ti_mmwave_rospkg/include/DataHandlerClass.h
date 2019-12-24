/**
 * @file DataHandlerClass.h
 * @author yzc
 * @brief 这个文件定义了DataUARTHandler类。雷达的串口通信使用了多线程双缓存的方式，将数据从串口读入并进行处理。
*/

#ifndef _DATA_HANDLER_CLASS_
#define _DATA_HANDLER_CLASS_


#include "mmWave.h"
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <boost/shared_ptr.hpp>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#define COUNT_SYNC_MAX 2

/**
 * @class DataUARTHandler DataHandlerClass.h
 * @brief 这个类通过串口通讯读到雷达数据，并将雷达数据处理成PointCloud2类型的数据结构。
 * 该类中参数的定义如下：
 * 变量名称 | 类型 | 描述
 * -------|------|-----
 * mmwData | mmwDataPacket | 传输过来的数据
 * dataBaudRate | int | 波特率
 * dataSerialPort |　char | 串口名称
 * maxAllowedElevationAngleDeg | int | 雷达获取数据最大垂直角度
 * maxAllowedAzimuthAngleDeg　| int | 雷达获取数据最大水平角度
 * currentBufp | vector<uint8_t> | 指向所处理数据的指针
 * nextBufp | vector<uint8_t> | 指向所读取数据的指针
 * countSync | int | 互斥锁所保护用来记录同步线程的数据
 * pingPongBuffers[2] | vector<uint8_t> | 读、写数据
 * countSync_mutex | pthread_mutex_t | 互斥锁用来保护记录同步线程的数据
 * nextBufp_mutex | pthread_mutex_t | 互斥锁用来保护处理数据
 * currentBufp_mutex | pthread_mutex_t | 互斥锁用来保护写入数据
 * countSync_max_cv　| pthread_cond_t |　阻塞交换线程的条件变量
 * read_go_cv　| pthread_cond_t | 阻塞读线程的条件变量
 * sort_go_cv | pthread_cond_t | 阻塞处理数据的条件变量
 * 
 */
class DataUARTHandler{
    
public:
    
    /*Constructor*/
   //void DataUARTHandler(ros::NodeHandle* nh) : currentBufp(&pingPongBuffers[0]) , nextBufp(&pingPongBuffers[1]) {}
    DataUARTHandler(ros::NodeHandle* nh);
    
    /**
     * @brief 设置URAT端口
    */
    void setUARTPort(char* mySerialPort);
    
    /**
     * @brief 设置波特率
    */
    void setBaudRate(int myBaudRate);

    /**
     * @brief 设置雷达最大垂直测量角度
     * 
    */
    void setMaxAllowedElevationAngleDeg(int myMaxAllowedElevationAngleDeg);
    
    /**
     * @brief 设置雷达最大水平测量角度
    */
    void setMaxAllowedAzimuthAngleDeg(int myMaxAllowedAzimuthAngleDeg);

    void setNodeHandle(ros::NodeHandle* nh);
      
    /**
     * @brief 启动三个线程分别来读数据、处理数据以及交换缓存数据
    */
    void start(void);
    
    static void* readIncomingData_helper(void *context);
    
    static void* sortIncomingData_helper(void *context);
    
    static void* syncedBufferSwap_helper(void *context);
    

    mmwDataPacket mmwData;

private:
    

    char* dataSerialPort;
    
    
    int dataBaudRate;
    

    int maxAllowedElevationAngleDeg;
    

    int maxAllowedAzimuthAngleDeg;
    
 
    int countSync;

    std::vector<uint8_t> pingPongBuffers[2];
    

    std::vector<uint8_t>* currentBufp;
   
    std::vector<uint8_t>* nextBufp;
    

    pthread_mutex_t countSync_mutex;
    

    pthread_mutex_t nextBufp_mutex;
    

    pthread_mutex_t currentBufp_mutex;
    
  
    pthread_cond_t countSync_max_cv;
    
  
    pthread_cond_t read_go_cv;
    

    pthread_cond_t sort_go_cv;
    
    /**
     * @brief 交换读取数据和处理数据的内存
    */
    void *syncedBufferSwap(void);
    /**
     * @brief 判断是不是magicWord
    */
    int isMagicWord(uint8_t last8Bytes[8]);
    

    void *readIncomingData(void);
    

    void *sortIncomingData(void);
    
    ros::NodeHandle* nodeHandle;
    
    ros::Publisher DataUARTHandler_pub;
    
};

#endif 
