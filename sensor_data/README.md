# SLAM-by-wwtx-lmk
传感器部分：
激光雷达数据读取和处理：订阅sensor_msg/PointCloud2话题，后续尝试订阅avia固态激光雷达数据类型；

IMU数据处理sensor_msgs/IMU，lmk，本周；

轮速里程计数据处理，本周；

GPS数据处理：本周；

特征提取部分：
激光雷达数据分类：模仿liosam：平面点和角点，可加入地面分割部分（模仿legoloam）；

IMU/轮速里程计部分对激光雷达数据的运动畸变的矫正；随时可参考，本周；
