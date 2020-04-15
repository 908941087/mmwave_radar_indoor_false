//
// Created by zgh on 2020/4/14.
//

#ifndef OCTOMAP_SERVER_MORPHOLOGYSERVER_H
#define OCTOMAP_SERVER_MORPHOLOGYSERVER_H
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>


class MorphologyServer {
public:
    MorphologyServer(const ros::NodeHandle private_nh_ = ros::NodeHandle("~"), const ros::NodeHandle &nh_ = ros::NodeHandle());
    virtual ~MorphologyServer();

    virtual void processCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input);

protected:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_nh_private;
    ros::Publisher  m_PCL2Pub;
    message_filters::Subscriber<sensor_msgs::PointCloud2>* m_PCL2Sub;
    bool m_open_or_close;
    double m_window_size;
};


#endif //OCTOMAP_SERVER_MORPHOLOGYSERVER_H
