# SLAM-by-wwtx-lmk
######################################2022年3月29日#######################################

传感器部分：
激光雷达数据读取和处理：订阅sensor_msg/PointCloud2话题，后续尝试订阅avia固态激光雷达数据类型；

IMU数据处理sensor_msgs/IMU，lmk，本周；

轮速里程计数据处理，本周；

GPS数据处理：本周；

特征提取部分：
激光雷达数据分类：模仿liosam：平面点和角点，可加入地面分割部分（模仿legoloam）；

IMU/轮速里程计部分对激光雷达数据的运动畸变的矫正；随时可参考，本周；

######################################2022年3月30日#######################################
工作：
1.编译sensor_lidar_point_node.cc文件名和节点名；
2.修改launch文件改成只启一个节点；
3.修改CMakeList.txt文件，
3.修改package文件
问题：
编译成功后：
使用launch文件启动时报错：
ERROR: cannot launch node of type [sensor_data_lidar/sensor_lidar_point_node]: Cannot locate node of type [sensor_lidar_point_node] in package [sensor_data_lidar]. Make sure file exists in package path and permission is set to executable (chmod +x)
未解决
但是在devel/lib/sensor_data_lidar/文件夹下可直接编译可执行文件，并能够成功运行代码！！
待解决问题：
使用launch文件启动节点。
