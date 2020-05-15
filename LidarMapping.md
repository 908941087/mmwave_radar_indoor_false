## 一、 配置激光雷达
### 1. 编译
进入ti_ws文件夹
``` 
    catkin_make 
```
### 2. 配置
   ```bash
   chmod 0777 src/ydlidar_ros/startup/*
   sudo sh src/ydlidar_ros/startup/initenv.sh
   ```
   **完成上述步骤之后，需要重新拔插雷达与电脑的USB连接**



## 二、 cartographer编译
进入carto_ws文件夹
``` 
    catkin_make_isolated --install --use-ninja
```



## 三、 cartographer运行（激光雷达）
### 1.启动激光雷达
进入ti_ws文件夹下
```
    source devel/setup.bash
    roslaunch ydlidar_ros X4.launch
```
### 2.运行cartographer
进入carto_ws文件夹下
```
    source install_isolated/setup.bash
    roslaunch cartographer_ros demo_revo_lds.launch
```



## 四、 Gmapping运行
### 1.启动激光雷达
进入ti_ws文件夹下
```
    source devel/setup.bash
    roslaunch ydlidar_ros X4.launch
```
### 2.运行gmapping
```
    rosrun gmapping slam_gmapping 