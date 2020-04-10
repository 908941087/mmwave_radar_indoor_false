1) 首先source /opt/ros/kinetic/setup.bash , 经过测试发现roscore不启动也可以运行

2) catkin_make source 什么的按照以前一样

3) 输入roslaunch turtlebot_bringup minimal.launch 启动雷达板和turtlebot

4) 输入roslaunch autonomous_explore_map_plan start_turtlebot_mapping_planning_control.launch 启动自动导航和建图

5) 合理control-C

----------------------------------------------------------------------------------
PS：本地编译时可能会在70-80%左右出现cmake error，出错停止后重新编译即可，其他错误请自行百度解决
