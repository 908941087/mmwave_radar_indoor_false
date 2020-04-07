# mmwave_radar_indoor_false

## Related library
### SOR
https://github.com/mgualti/PointCloudsPython


## Simulation Environment
1. <启动roscore>
> roscore
2. 启动minimal.launch
> roslaunch turtlebot_bringup minimal.launch
3. 启动过滤节点
> rosrun py_interface listener.py
4. 启动建图相关节点
> roslaunch turtlebot_mmwave_launchers radar_mapping.launch
5. 启动rviz (rviz 可-d选择文件夹下mapping的rviz配置文件)
> roslaunch ti_mmwave_rospkg rviz_6843_2d.launch
6. 启动回放数据 (-l 为循环播放，按空格可暂停)
> rosbag play input.bag -l

# static navigation

​	1. 启动minimal.launch

> roslaunch turtlebot_bringup minimal.launch

​	2. 启动过滤节点

> rosrun py_interface listener.py

3. 启动建图相关节点

> roslaunch turtlebot_mmwave_launchers radar_navigation.launch

4. 启动rviz 

> rviz rosrun rviz rviz -d ~/catkin_ws/src/turtlebot_mmwave_launchers/launch/navigation_visualization.rviz

# auto navigation

​	1. 启动minimal.launch

> roslaunch turtlebot_bringup minimal.launch

​	2. 启动过滤节点

> rosrun py_interface listener.py

3. 启动建图相关节点

> roslaunch turtlebot_mmwave_launchers radar_navigation_auto.launch

4. 启动rviz 

> rviz rosrun rviz rviz -d ~/catkin_ws/src/turtlebot_mmwave_launchers/launch/navigation_visualization.rviz


录制数据：
rosbag record -a
查看相关节点话题信息命令 :
rosnode list
rosnode info /
rostopic list
rostopic info /节点名
rostopic echo / 节点名
