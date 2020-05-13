# YDLiDAR Build Manual

1. 运行以下命令，将下面的项目下载到ti_ws的src目录下

   ```bash
   $cd ti_ws/src
   $git clone https://github.com/YDLIDAR/ydlidar_ros
   ```

   

2. Build

   ```bash
   $cd ..
   $catkin_make
   ```

   

3. 设置环境变量

   ```bash
   $echo "source ~/.../ti_ws/devel/setup.bash" >> ~/.bashrc # 路径需要根据自己文件夹路径更改
   $source ~/.bashrc
   ```

   

4. 创建串口别名

   ```bash
   $chmod 0777 src/ydlidar_ros/startup/*
   $sudo sh src/ydlidar_ros/startup/initenv.sh
   ```

   **完成上述步骤之后，需要重新拔插雷达与电脑的USB连接**

   

5. 启动雷达

   在新终端中

   ```bash
   $roscore
   ```

   在新终端中

   ```bash
   $roslaunch ydlidar_ros S2.launch
   ```

   