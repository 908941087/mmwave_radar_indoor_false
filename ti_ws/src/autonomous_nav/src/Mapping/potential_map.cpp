#include <iostream>
#include <string>
#include <csignal>
#include <sstream>
#include <cmath>
#include <climits>

# include <algorithm>
# include <vector>
# include <list>
# include <deque>

using namespace std;

//ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

//Messages
#include <std_msgs/Int16.h>
#include "std_msgs/String.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>

#include <autonomous_nav/PotentialGrid.h>

struct Pixel{
    int x, y;
    Pixel(int x_in, int y_in){
        x = x_in; y = y_in;
    }
};

class PotentialMapMaker{
public:
    PotentialMapMaker();
    //virtual ~PotentialMapMaker();

    void projectedMapCallback(const nav_msgs::OccupancyGrid& msg);
    void odometryCallback(const nav_msgs::Odometry& msg);

private:
    boost::mutex mtx;
    ros::NodeHandle node_handle;

    //Robot odometry tracking
    ros::Subscriber robot_pos_sub;
    double robot_x, robot_y; 

    //Potential Maps
    ros::Subscriber projected_map_sub;
    ros::Publisher potential_map_pub;

    vector<vector<int>> projected_map;
    int robot_pixel_i, robot_pixel_j;
    autonomous_nav::PotentialGrid last_potential_map;

    // add params
    string param_odom, param_polygon_map;
    int param_inflation_radius;

};

PotentialMapMaker::PotentialMapMaker(){
    //Initalize some variables
    robot_x = 0; robot_y = 0;
    last_potential_map = autonomous_nav::PotentialGrid();

    //read the params from potential_map_param.yaml
    ros::NodeHandle private_nh("~");
    private_nh.param("potential_odom", param_odom, string("/odom"));
    private_nh.param("potential_polygon_map", param_polygon_map, string("/map"));
    private_nh.param("potential_inflation_radius", param_inflation_radius, 8);

    //Node I/O
    robot_pos_sub = node_handle.subscribe(param_odom, 10, &PotentialMapMaker::odometryCallback, this);
    
    //Potential Map Making
    projected_map_sub = node_handle.subscribe(param_polygon_map, 1, &PotentialMapMaker::projectedMapCallback, this);
    potential_map_pub = node_handle.advertise<autonomous_nav::PotentialGrid>("potential_map", 1);
}

void PotentialMapMaker::odometryCallback(const nav_msgs::Odometry& msg){
    mtx.lock();
    
    robot_x = msg.pose.pose.position.x;
    robot_y = msg.pose.pose.position.y;

    mtx.unlock();
}

//#####################################################################
// POTENTIAL MAP CREATION
void PotentialMapMaker::projectedMapCallback(const nav_msgs::OccupancyGrid& msg){
    mtx.lock();

    //Resize the projected_map variable for new size
    projected_map.resize(msg.info.width);
    for(int i = 0; i < projected_map.size(); i++)
        projected_map[i].resize(msg.info.height, 0);
    
    //Copy map into 2D array and initialize inflation queue
    //IN: UNKNOWN -1, FREE 0, OCCUPIED 100
    //OUT: UNKNOWN 1, FREE 0, OCCUPIED 2
    deque<Pixel> inflation_queue;
    for(int i = 0; i < projected_map.size(); i++){ //msg.info.width -> columns
        for(int j = 0; j < projected_map[0].size(); j++){//msg.info.height -> rows

            //Obstacle
            if (msg.data[projected_map.size()*j + i] == 100){ 
                projected_map[i][j] = 2;
                //if obstacle add to queue for inflation
                inflation_queue.push_back({i, j});

            //Unknown    
            }else if(msg.data[i +  projected_map.size()*j] == -1){ 
                projected_map[i][j] = 1;

            //Free    
            }else{ 
                projected_map[i][j] = 0;
            }
        }
    }

    //1. INFLATE OBSTACLES
    // ROS_INFO("start cal potential_map");
    int inflate = param_inflation_radius;
    deque<Pixel> inflation_queue2;

    for(int n = 0; n < inflate; n++){
        inflation_queue2 = deque<Pixel>();

        while(!inflation_queue.empty()){
            for(int i = std::max(0, inflation_queue.front().x-1); i < std::min(inflation_queue.front().x+2, (int)msg.info.width); i++){
                for(int j = std::max(0, inflation_queue.front().y-1); j < std::min(inflation_queue.front().y+2, (int)msg.info.height); j++){
                    if(projected_map[i][j] <= 1){
                        projected_map[i][j] = 2;
                        inflation_queue2.push_back(Pixel(i, j));
                    }
                }
            }
            inflation_queue.pop_front();
        }
        inflation_queue = inflation_queue2;
    }

    //2. COMPUTE POTENTIAL
    //Put 3 in robot position (have to convert to pixel)
    robot_pixel_i = round((robot_x - msg.info.origin.position.x)/msg.info.resolution);
    robot_pixel_j = round((robot_y - msg.info.origin.position.y)/msg.info.resolution);

    if(robot_pixel_i < 0 || robot_pixel_i > msg.info.width ||
        robot_pixel_j < 0 || robot_pixel_j > msg.info.height){
        ROS_WARN("Robot outside of projected_map");
        mtx.unlock();
        return;
    }

    projected_map[robot_pixel_i][robot_pixel_j] = 3;

    //Initialize queue with robot position
    deque<Pixel> queue;
    queue.push_back(Pixel(robot_pixel_i, robot_pixel_j));

    while(!queue.empty()){
        for(int i = std::max(0, queue.front().x-5); i < std::min(queue.front().x+5, (int)msg.info.width); i++){
            for(int j = std::max(0, queue.front().y-5); j < std::min(queue.front().y+5, (int)msg.info.height); j++){
                //Free
                if(projected_map[i][j] == 0){
                    projected_map[i][j] = projected_map[queue.front().x][queue.front().y] + 1;
                    queue.push_back(Pixel(i,j));
                }
            }
        }
        queue.pop_front();
    }

    //Create message for potential map based on original occupancy grid msg
    last_potential_map = autonomous_nav::PotentialGrid();
    last_potential_map.header = msg.header;
    last_potential_map.info = msg.info;

    for(int j = 0; j < projected_map[0].size(); j++)
        for(int i = 0; i < projected_map.size(); i++)
            last_potential_map.data.push_back( projected_map[i][j]);

    potential_map_pub.publish(last_potential_map);  
    
    mtx.unlock();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "potential_map");
    
    PotentialMapMaker pmm;

    ros::spin();

    return (0);
}
