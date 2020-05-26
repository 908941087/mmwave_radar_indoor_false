//
// Created by zgh on 2020/5/25.
//
#include<transparent_layers/transparent_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(transparent_layer_namespace::TransparentLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;


namespace transparent_layer_namespace
{

    TransparentLayer::TransparentLayer() {}

    void TransparentLayer::onInitialize()
    {
        ros::NodeHandle nh("~/" + name_), g_nh;

        std::string obstacle_topic;
        nh.param("obstacle_topic", obstacle_topic, std::string("transparent_obstacle"));

        current_ = true;
        default_value_ = NO_INFORMATION;
        matchSize();

        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
                &TransparentLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        // Only resubscribe if topic has changed
        if (obstacle_sub_.getTopic() != ros::names::resolve(obstacle_topic)) {
            // we'll subscribe to the latched topic that the map server uses
            ROS_INFO("Requesting the transparent obstacles...");
            obstacle_sub_ = g_nh.subscribe(obstacle_topic, 1, &TransparentLayer::handlePolygon, this);
            obstacle_received_ = false;
        }
    }

    void TransparentLayer::matchSize()
    {
        Costmap2D* master = layered_costmap_->getCostmap();
        resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
                  master->getOriginX(), master->getOriginY());
    }

    void TransparentLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
    {
        enabled_ = config.enabled;
    }

    void TransparentLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                   double* min_y, double* max_x, double* max_y)
    {
        if (!enabled_)
            return;
    }

    void TransparentLayer::handlePolygon(const geometry_msgs::PolygonConstPtr &polygonPtr)
    {
        if(polygonPtr == NULL || (*polygonPtr).points.size() <= 1) return;
        mark_lines.clear();
        for(int i = 0; i < (*polygonPtr).points.size(); ++i) {
            geometry_msgs::Point32 t_point = (*polygonPtr).points[i];
            std::vector<double> t_vec(2, 0.0);
            t_vec[0] = t_point.x;
            t_vec[1] = t_point.y;
            mark_lines.push_back(t_vec);
        }
    }

    void TransparentLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                  int max_j)
    {
        if (!enabled_)
            return;
        unsigned int mx1;
        unsigned int my1;
        unsigned int mx2;
        unsigned int my2;

        if (mark_lines.empty()) return;
        for(int i = 0; i < mark_lines.size(); i+=2) {
            // TODO: Find the correct tf.
            if(master_grid.worldToMap(mark_lines[i][0], mark_lines[i][1], mx1, my1)
            && master_grid.worldToMap(mark_lines[i+1][0], mark_lines[i+1][1], mx2, my2)){
                std::vector<std::vector<double> > line_points;
                genLinePoints(line_points, mx1, my1, mx2, my2);
                for(int  i = 0; i < line_points.size(); ++i){
                    master_grid.setCost(line_points[i][0], line_points[i][1], LETHAL_OBSTACLE);
                }
            }
        }
    }

    void TransparentLayer::genLinePoints(std::vector<std::vector<double> > &pointsOut, double mx1, double my1, double mx2, double my2, uint32_t step) {
        double deltaX, deltaY;
        if(step < 0) return;
        deltaX = (mx2-mx1) / step;
        deltaY = (my2-my1) / step;
        for(int i = 0; i < step; ++i){// Step not include in loop
            std::vector<double> t_vec(2, 0.0);
            t_vec[0] = deltaX*step + mx1;
            t_vec[1] = deltaY*step + my1;
            pointsOut.push_back(t_vec);
        }
    }
} // end namespace
