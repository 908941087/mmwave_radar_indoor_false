## һ�� ���ü����״�
### 1. ����
����ti_ws�ļ���
``` 
    catkin_make 
```
### 2. ����
   ```bash
   chmod 0777 src/ydlidar_ros/startup/*
   sudo sh src/ydlidar_ros/startup/initenv.sh
   ```
   **�����������֮����Ҫ���°β��״�����Ե�USB����**



## ���� cartographer����
����carto_ws�ļ���
``` 
    catkin_make_isolated --install --use-ninja
```



## ���� cartographer���У������״
### 1.���������״�
����ti_ws�ļ�����
```
    source devel/setup.bash
    roslaunch ydlidar_ros X4.launch
```
### 2.����cartographer
����carto_ws�ļ�����
```
    source install_isolated/setup.bash
    roslaunch cartographer_ros demo_revo_lds.launch
```



## �ġ� Gmapping����
### 1.���������״�
����ti_ws�ļ�����
```
    source devel/setup.bash
    roslaunch ydlidar_ros X4.launch
```
### 2.����gmapping
```
    rosrun gmapping slam_gmapping 