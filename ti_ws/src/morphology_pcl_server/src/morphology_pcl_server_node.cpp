//
// Created by zgh on 2020/4/14.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/morphological_filter.h>
#include <pcl/filters/extract_indices.h>
#include "../include/MorphologyServer.h"


pcl::PointIndicesPtr no_ground (new pcl::PointIndices);
ros::Publisher pub;

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "mapping_morphology");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    MorphologyServer server(private_nh, nh);
    ros::spinOnce();

    try{
        ros::spin();
    }catch(std::runtime_error& e){
        ROS_ERROR("MorphologyServer exception: %s", e.what());
        return -1;
    }

    return 0;
}