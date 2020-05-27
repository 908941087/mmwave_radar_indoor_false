//
// Created by zgh on 2020/5/25.
//

#ifndef TRANSPARENT_LAYERS_TRANSPARENT_LAYER_H
#define TRANSPARENT_LAYERS_TRANSPARENT_LAYER_H

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Polygon.h>

typedef boost::shared_ptr< ::geometry_msgs::Polygon > PolygonPtr;

namespace transparent_layer_namespace
{

    class TransparentLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
    {
    public:
        TransparentLayer();

        virtual void onInitialize();
        virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                                  double* max_y);
        virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
        virtual void matchSize();

        static void genLinePoints(std::vector<std::vector<double> > &pointsOut, double mx1, double my1, double mx2, double my2, uint32_t step = 50);

    private:
        void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
        void handlePolygon(const geometry_msgs::PolygonConstPtr& polygonPtr);
        std::vector<std::vector<double> > mark_lines;
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

        ros::Subscriber obstacle_sub_, map_update_sub_;
        bool obstacle_received_;
    };
}

#endif //TRANSPARENT_LAYERS_TRANSPARENT_LAYER_H
