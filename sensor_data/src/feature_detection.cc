/*
 * Copyright 2020 The Project Author: lixiang
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <map>
#include <vector>
#include <chrono>

#define max_scan_count 1500 // 雷达数据个数的最大值



/**
 * 激光点曲率
*/
struct smoothness_t{ 
    float value; // 曲率值
    size_t ind;  // 激光点一维索引
};

/**
 * 曲率比较函数，从小到大排序
*/
struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

// struct smoothness_t
// {
//     float value;
//     size_t index;
// };

// // 排序的规则,从小到大进行排序
// struct by_value
// {
//     bool operator()(smoothness_t const &left, smoothness_t const &right)
//     {
//         return left.value < right.value;
//     }
// };

// 声明一个类
// class LaserScan
// {
// private:
//     ros::NodeHandle node_handle_;           // ros中的句柄
//     ros::NodeHandle private_node_;          // ros中的私有句柄
//     ros::Subscriber laser_scan_subscriber_; // 声明一个Subscriber
//     ros::Publisher feature_scan_publisher_; // 声明一个Publisher

//     float edge_threshold_; // 提取角点的阈值

// public:
//     LaserScan();
//     ~LaserScan();
//     void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
// };
class PointCloud2
{
private:
    // ros::NodeHandle node_handle_;           // ros中的句柄
    // ros::NodeHandle private_node_;          // ros中的私有句柄
    // ros::Subscriber laser_scan_subscriber_; // 声明一个Subscriber
    ros::NodeHandle node_handle_;           // ros中的句柄
    ros::NodeHandle private_node_;          // ros中的私有句柄
    ros::Subscriber point_cloud_subscriber_; // 声明一个Subscriber
    ros::Publisher feature_point_publisher_;
    ros::Publisher feature_point_publisher_Corner;
    ros::Publisher feature_point_publisher_Surface;


    float edge_threshold_;//提取角点的阈值
    float surf_Threshold_;

public:
    // LaserScan();
    // ~LaserScan();
    // void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
    PointCloud2();
    ~PointCloud2();
    void ScanCallback(const sensor_msgs::PointCloud2::ConstPtr &point_msg);
};
// 构造函数
// LaserScan::LaserScan() : private_node_("~")
// {
//     // \033[1;32m，\033[0m 终端显示成绿色
//     ROS_INFO_STREAM("\033[1;32m----> Feature Extraction Started.\033[0m");

//     // 将雷达的回调函数与订阅的topic进行绑定
//     laser_scan_subscriber_ = node_handle_.subscribe("laser_scan", 1, &LaserScan::ScanCallback, this);
//     // 将提取后的点发布到 feature_scan 这个topic
//     feature_scan_publisher_ = node_handle_.advertise<sensor_msgs::LaserScan>("feature_scan", 1, this);

//     // 将提取角点的阈值设置为1.0
//     edge_threshold_ = 1.0;
// }

// LaserScan::~LaserScan()
// {
// }
PointCloud2::PointCloud2() : private_node_("~")
{
    // \033[1;32m，\033[0m 终端显示成绿色
    ROS_INFO_STREAM("\033[1;32m----> Feature Extraction Started.\033[0m");    // 将雷达的回调函数与订阅的topic进行绑定
    point_cloud_subscriber_ = node_handle_.subscribe("velodyne_points", 1, &PointCloud2::PointCallback, this);//话题名为velodyne_points
    feature_point_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>("feature_points", 1, this);
    // 发布当前激光帧的角点点云
    feature_point_publisher_Corner = node_handle_.advertise<sensor_msgs::PointCloud2>("feature_points/cloud_corner", 1,this);
    // 发布当前激光帧的面点点云
    feature_point_publisher_Surface = node_handle_.advertise<sensor_msgs::PointCloud2>("feature_points/cloud_surface", 1,this);
        
    edge_threshold_=0.1;//提取角阈值
    surf_Threshold_=0.1
}
PointCloud2::~PointCloud2()
{
}
// 回调函数
void PointCloud2::ScanCallback(const sensor_msgs::PointCloud2::ConstPtr &point_msg)
{
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

    std::vector<smoothness_t> point_smoothness_(max_scan_count); // 存储每个点的曲率与索引
    float *point_curvature_ = new float[max_scan_count];         // 存储每个点的曲率
    
    sensor_msgs::pointCloud2 cloudInfo;
    cloudInfo=*point_msg;
    pcl::PointCloud<PointType>::Ptr extractedCloud;
    pcl::fromROSMsg(cloudInfo->cloud_deskewed, *extractedCloud); //转化为pcl的pointcloud的格式



    // std::map<int, int> map_index;   // 有效点的索引 对应的 scan实际的索引
    // int count = 0;                  // 有效点的索引
    // float new_scan[max_scan_count]; // 存储scan数据的距离值

    std::map<int, int> map_index;   // 有效点的索引 对应的 scan实际的索引
    int count = 0;                  // 有效点的索引
    float new_scan[max_scan_count]; // 存储scan数据的距离值


    // 通过ranges中数据的个数进行雷达数据的遍历
    // int scan_count = scan_msg->ranges.size();
    int scan_count = extractedCloud->points.size();

    // ROS_INFO_STREAM("scan_count: " << scan_count);

    // 去处inf或者nan点,保存有效点
    // for (int i = 0; i < scan_count; i++)
    // {
    //     if (!std::isfinite(scan_msg->ranges[i]))
    //     {
    //         // std::cout << " " << i << " " << scan_msg->ranges[i];
    //         continue;
    //     }

    //     // 这点在原始数据中的索引为i，在new_scan中的索引为count
    //     map_index[count] = i;
    //     // new_scan中保存了有效点的距离值
    //     new_scan[count] = scan_msg->ranges[i];
    //     count++;
    // }
    for (int i = 0; i < scan_count; i++)
    {
        if (!std::isfinite(extractedCloud->points[i]))
        {
            // std::cout << " " << i << " " << scan_msg->ranges[i];
            continue;
        }

        // 这点在原始数据中的索引为i，在new_scan中的索引为count
        map_index[count] = i;
        // new_scan中保存了有效点的距离值
        new_scan[count] = extractedCloud->points[i];
        count++;
    }
    // std::cout << "count: " << count << std::endl;

    // 计算曲率值, 通过当前点前后5个点距离值的偏差程度来代表曲率
    // 如果是球面, 则当前点周围的10个点的距离之和 减去 当前点距离的10倍 应该等于0
    int *pointLabel;
    int *cloudNeighborPicked;
    pointLabel = new int[max_scan_count];
    cloudNeighborPicked = new int[max_scan_count];

    for (int i = 5; i < count - 5; i++)
    {
        float diff_range = cloudInfo.pointRange[i-5] + cloudInfo.pointRange[i-4]
                            + cloudInfo.pointRange[i-3] + cloudInfo.pointRange[i-2]
                            + cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i] * 10
                            + cloudInfo.pointRange[i+1] + cloudInfo.pointRange[i+2]
                            + cloudInfo.pointRange[i+3] + cloudInfo.pointRange[i+4]
                            + cloudInfo.pointRange[i+5];    
        // diffX * diffX + diffY * diffY
        // 距离差值平方作为曲率
        point_curvature_[i] = diff_range * diff_range;
        cloudNeighborPicked[i] = 0;
        pointLabel[i] = 0;
        point_smoothness_[i].value = point_curvature_[i];
        point_smoothness_[i].index = i;
    }

    // 声明一个临时的sensor_msgs::LaserScan变量,用于存储特征提取后的scan数据,并发布出去,在rviz中进行显示
    // sensor_msgs::PointCloud2 point_publisher_Corner=cloudInfo;
    // sensor_msgs::PointCloud2 point_publisher_Surface=cloudInfo;

        // 当前激光帧角点点云集合
    pcl::PointCloud<PointType>::Ptr point_publisher_Surface;
    // 当前激光帧平面点点云集合
    pcl::PointCloud<PointType>::Ptr surfaceCloud;

    pcl::VoxelGrid<PointType> downSizeFilter;
    // corner_scan.header = scan_msg->header;
    // corner_scan.angle_min = scan_msg->angle_min;
    // corner_scan.angle_max = scan_msg->angle_max;
    // corner_scan.angle_increment = scan_msg->angle_increment;
    // corner_scan.range_min = scan_msg->range_min;
    // corner_scan.range_max = scan_msg->range_max;
        int cloudSize = extractedCloud->points.size();
        // mark occluded points and parallel beam points
    /**
     * 标记属于遮挡、平行两种情况的点，不做特征提取
    */
        for (int i = 5; i < scan_count - 6; ++i)
        {
            // 当前点和下一个点的range值
            float depth1 = cloudInfo.pointRange[i];
            float depth2 = cloudInfo.pointRange[i+1];
            // 两个激光点之间的一维索引差值，如果在一条扫描线上，那么值为1；如果两个点之间有一些无效点(距离筛选等)被剔除了，可能会比1大，但不会特别大
            // 如果恰好前一个点在扫描一周的结束时刻，下一个点是另一条扫描线的起始时刻，那么值会很大
            int columnDiff = std::abs(int(cloudInfo.pointColInd[i+1] - cloudInfo.pointColInd[i]));

            // 两个点在同一扫描线上，且距离相差大于0.3，认为存在遮挡关系（也就是这两个点不在同一平面上，如果在同一平面上，距离相差不会太大）
            // 远处的点会被遮挡，标记一下该点以及相邻的5个点，后面不再进行特征提取
            if (columnDiff < 10){
                
                if (depth1 - depth2 > 0.3){
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }else if (depth2 - depth1 > 0.3){
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }
            
            // 用前后相邻点判断当前点所在平面是否与激光束方向平行
            float diff1 = std::abs(float(cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i]));
            float diff2 = std::abs(float(cloudInfo.pointRange[i+1] - cloudInfo.pointRange[i]));

            // 平行则标记一下
            if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
                cloudNeighborPicked[i] = 1;
        }
    // 对float[] 进行初始化
    // corner_scan.ranges.resize(max_scan_count);

        point_publisher_Corner->clear();
        point_publisher_Surface->clear();
    // 进行角点的提取,将完整的scan分成6部分,每部分提取20个角点
    // for (int j = 0; j < 6; j++)
    // {
    //     int start_index = (0 * (6 - j) + count * j) / 6;
    //     int end_index = (0 * (5 - j) + count * (j + 1)) / 6 - 1;
    //     // std::cout << "start_index: " << start_index << " end_index: " << end_index << std::endl;

    //     if (start_index >= end_index)
    //         continue;

    //     // 将这段点云按照曲率从小到大进行排序
    //     std::sort(point_smoothness_.begin() + start_index,
    //               scan_smoothness_.begin() + end_index, by_value());

    //     int largestPickedNum = 0;
    //     // 最后的点 的曲率最大，如果满足条件，就是角点
    //     for (int k = end_index; k >= start_index; k--)
    //     {
    //         int index = scan_smoothness_[k].index;
    //         if (scan_smoothness_[k].value > edge_threshold_)
    //         {
    //             // 每一段最多只取20个角点
    //             largestPickedNum++;
    //             if (largestPickedNum <= 20)
    //             {
    //                 corner_scan.ranges[map_index[index]] = scan_msg->ranges[map_index[index]];
    //             }
    //             else
    //             {
    //                 break;
    //             }
    //         }
    //     }
    // }

        pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

        // 遍历扫描线
        for (int i = 0; i < N_SCAN; i++)
        {
            surfaceCloudScan->clear();

            //// 将一条扫描线扫描一周的点云数据，划分为6段，每段分开提取有限数量的特征，保证特征均匀分布
            for (int j = 0; j < 6; j++)
            {
                // 每段点云的起始、结束索引；startRingIndex为扫描线起始第5个激光点在一维数组中的索引
                /*********************************************************
                // 每段点云的起始、结束索引；startRingIndex为扫描线起始第5个激光点在一维数组中的索引
                //注意：所有的点云在这里都是以"一维数组"的形式保存
                //startRingIndex和 endRingIndex 在imageProjection.cpp中的 cloudExtraction函数里被填入
                //假设 当前ring在一维数组中起始点是m，结尾点为n（不包括n），那么6段的起始点分别为：
                // m + [(n-m)/6]*j   j从0～5
                // 化简为 [（6-j)*m + nj ]/6
                // 6段的终止点分别为：
                // m + (n-m)/6 + [(n-m)/6]*j -1  j从0～5,-1是因为最后一个,减去1
                // 化简为 [（5-j)*m + (j+1)*n ]/6 -1
                //这块不必细究边缘值到底是不是划分的准（例如考虑前五个点是不是都不要，还是说只不要前四个点），
                 *
                *************************************************** * */
                int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
                int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

                if (sp >= ep)
                    continue;

                // 按照曲率从小到大排序点云
                std::sort(point_smoothness_.begin()+sp, point_smoothness_.begin()+ep, by_value());

                // 按照曲率从大到小遍历
                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--)
                {
                    // 激光点的索引
                    int ind = point_smoothness_[k].ind;
                    // 当前激光点还未被处理，且曲率大于阈值，则认为是角点
                    if (cloudNeighborPicked[ind] == 0 && point_curvature_[ind] > edge_threshold_)
                    {
                        // 每段只取20个角点，如果单条扫描线扫描一周是1800个点，则划分6段，每段300个点，从中提取20个角点
                        largestPickedNum++;
                        if (largestPickedNum <= 20){
                            // 标记为角点
                            pointLabel[ind] = 1;
                            // 加入角点点云
                            point_publisher_Corner->push_back(extractedCloud->points[ind]);
                        } else {
                            break;
                        }

                        // 标记已被处理
                        cloudNeighborPicked[ind] = 1;
                        // 同一条扫描线上后5个点标记一下，不再处理，避免特征聚集
                        for (int l = 1; l <= 5; l++)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            //// 这说明不是一个 ring
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        // 同一条扫描线上前5个点标记一下，不再处理，避免特征聚集
                        for (int l = -1; l >= -5; l--)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                // 按照曲率从小到大遍历
                for (int k = sp; k <= ep; k++)
                {
                    // 激光点的索引
                    int ind = point_smoothness_[k].ind;
                    // 当前激光点还未被处理，且曲率小于阈值，则认为是平面点
                    if (cloudNeighborPicked[ind] == 0 && point_curvature_[ind] < surf_Threshold_)
                    {
                        // 标记为平面点
                        pointLabel[ind] = -1;
                        // 标记已被处理
                        cloudNeighborPicked[ind] = 1;

                        // 同一条扫描线上后5个点标记一下，不再处理，避免特征聚集
                        for (int l = 1; l <= 5; l++) {

                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        // 同一条扫描线上前5个点标记一下，不再处理，避免特征聚集
                        for (int l = -1; l >= -5; l--) {

                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                // 平面点和未被处理的点，都认为是平面点，加入平面点云集合
                for (int k = sp; k <= ep; k++)
                {
                    if (pointLabel[k] <= 0){
                        surfaceCloudScan->push_back(extractedCloud->points[k]);
                    }
                }
            }

            // 平面点云降采样
            surfaceCloudScanDS->clear();
            downSizeFilter.setInputCloud(surfaceCloudScan);
            downSizeFilter.filter(*surfaceCloudScanDS);

            // 加入平面点云集合
            // 用surfaceCloudScan来装数据，然后放到downSizeFilter里，
            // 再用downSizeFilter进行.filter()操作，把结果输出到*surfaceCloudScanDS里。
            // 最后把DS装到surfaceCloud中。DS指的是DownSample。
            // 同样角点（边缘点）则没有这样的操作，直接就用cornerCloud来装点云。
            *surfaceCloud += *surfaceCloudScanDS;
        }

    // 将提取后的scan数据发布出去
        // cloudInfo.cloud_corner  = publishCloud(&feature_point_publisher_Corner,  point_publisher_Corner,  cloudHeader.stamp, lidarFrame);
        // cloudInfo.cloud_surface = publishCloud(&pubSurfacePoints, surfaceCloud, cloudHeader.stamp, lidarFrame);
        // 发布当前激光帧点云信息，加入了角点、面点点云数据，发布给mapOptimization
        feature_point_publisher_.publish(cloudInfo);
        feature_point_publisher_Corner.publish(point_publisher_Corner);
        feature_point_publisher_Surface.publish(surfaceCloud);

    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
    // std::cout<<"处理一次数据用时: "<< time_used.count() << " 秒。" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lesson1_feature_detection_node"); // 节点的名字
    PointCloud2 PointCloud2
    ;

    ros::spin(); // 程序执行到此处时开始进行等待，每次订阅的消息到来都会执行一次ScanCallback()
    return 0;
}