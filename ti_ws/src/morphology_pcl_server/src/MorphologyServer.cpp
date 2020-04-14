//
// Created by zgh on 2020/4/14.
//
#include <pcl/point_cloud.h>
#include <pcl/filters/morphological_filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/impl/point_types.hpp>
#include "../include/MorphologyServer.h"

MorphologyServer::MorphologyServer(ros::NodeHandle private_nh_, const ros::NodeHandle &nh_)
: m_nh(nh_),
  m_nh_private(private_nh_),
  m_PCL2Sub(NULL),
  m_open_or_close(true),
  m_window_size(0.2)
{
    double window_size;
    m_nh_private.param("open_or_close", m_open_or_close, true);
    m_nh_private.param("window_size", window_size, 0.2);
    m_PCL2Pub = m_nh.advertise<sensor_msgs::PointCloud2>("output", 1, true);
    m_PCL2Sub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "input", 5);
    m_window_size = window_size;
    m_PCL2Sub->registerCallback(boost::bind(&MorphologyServer::processCloudCallback, this, _1));
}

MorphologyServer::~MorphologyServer() {
    if (m_PCL2Sub){
        delete m_PCL2Sub;
        m_PCL2Sub = NULL;
    }
}

void MorphologyServer::processCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input) {

    pcl::PCLPointCloud2Ptr cloud_input(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2 cloud_filtered;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    // 转化为PCL中的点云的数据格式
    pcl_conversions::toPCL(*input, *cloud_input);
    pcl::fromPCLPointCloud2(*cloud_input, *xyz_cloud_in);
     //Data processing
    int morph_op = pcl::MORPH_OPEN;
     if(!m_open_or_close) {
         morph_op = pcl::MORPH_CLOSE;
     }

    pcl::applyMorphologicalOperator(xyz_cloud_in, 0.2, morph_op, *xyz_cloud_out);

    // Publish the data.
    pcl::toPCLPointCloud2(*xyz_cloud_out, cloud_filtered);
    m_PCL2Pub.publish (cloud_filtered);

}