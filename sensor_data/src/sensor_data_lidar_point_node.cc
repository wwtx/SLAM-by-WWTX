#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>//改成三维激光雷达的格式，注意这里是sensor_msgs下的格式
#include <sensor_msgs/point_cloud_conversion.h>


// 声明一个类
class PointCloud2
{
private:
    // ros::NodeHandle node_handle_;           // ros中的句柄
    // ros::NodeHandle private_node_;          // ros中的私有句柄
    // ros::Subscriber laser_scan_subscriber_; // 声明一个Subscriber
    ros::NodeHandle node_handle_;           // ros中的句柄
    ros::NodeHandle private_node_;          // ros中的私有句柄
    ros::Subscriber point_cloud_subscriber_; // 声明一个Subscriber

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
//     ROS_INFO_STREAM("LaserScan initial.");
//     // 将雷达的回调函数与订阅的topic进行绑定
//     laser_scan_subscriber_ = node_handle_.subscribe("laser_scan", 1, &LaserScan::ScanCallback, this);
// }
PointCloud2::PointCloud2() : private_node_("~")
{
    ROS_INFO_STREAM("PointCloud2.");
    // 将雷达的回调函数与订阅的topic进行绑定
    point_cloud_subscriber_ = node_handle_.subscribe("point_cloud", 1, &PointCloud2::PointCallback, this);
}
PointCloud2::~PointCloud2()
{
}
// LaserScan::~LaserScan()
// {
// }
// 回调函数
// void LaserScan::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
// {
//     ROS_INFO_STREAM(
//         "seqence: " << scan_msg->header.seq << 
//         ", time stamp: " << scan_msg->header.stamp << 
//         ", frame_id: " << scan_msg->header.frame_id << 
//         ", angle_min: " << scan_msg->angle_min << 
//         ", angle_max: " << scan_msg->angle_max << 
//         ", angle_increment: " << scan_msg->angle_increment << 
//         ", time_increment: " << scan_msg->time_increment << 
//         ", scan_time: " << scan_msg->scan_time << 
//         ", range_min: " << scan_msg->range_min << 
//         ", range_max: " << scan_msg->range_max << 
//         ", range size: " << scan_msg->ranges.size() << 
//         ", intensities size: " << scan_msg->intensities.size());

//     // 第5个点的欧式坐标为
//     double range = scan_msg->ranges[4];
//     double angle = scan_msg->angle_min + scan_msg->angle_increment * 4;
//     double x = range * cos(angle);
//     double y = range * sin(angle);

//     ROS_INFO_STREAM(
//         // 第5个数据点对应的极坐标为: 
//         "range = " << range << ", angle = " << angle << 
//         // 第5个数据点对应的欧式坐标为: 
//         ", x = " << x << ", y = " << y
//     );

//     // 通过ranges中数据的个数进行雷达数据的遍历
//     // for (int i = 0; i < scan_msg->ranges.size(); i++)
//     // {

//     // }

// }
// header:  // 点云的头信息
//   seq: 963 //
//   stamp:  // 时间戳
//     secs: 1541143772
//     nsecs: 912011000
//   frame_id: "/camera_init"
// height: 1   // If the cloud is unordered, height is 1  如果cloud 是无序的 height 是 1
// width: 852578  //点云的长度
// fields:  //  sensor_msgs/PointField[] fields 
//   - 
//     name: "x"
//     offset: 0
//     datatype: 7 	// 	uint8 INT8    = 1
// 			//	uint8 UINT8   = 2
// 			//	uint8 INT16   = 3
// 			//	uint8 UINT16  = 4
// 			//	uint8 INT32   = 5
// 			//	uint8 UINT32  = 6
// 			//	uint8 FLOAT32 = 7
// 			//	uint8 FLOAT64 = 8
//     count: 1
//   - 
//     name: "y"
//     offset: 4
//     datatype: 7
//     count: 1
//   - 
//     name: "z"
//     offset: 8
//     datatype: 7
//     count: 1
//   - 
//     name: "intensity"
//     offset: 16
//     datatype: 7
//     count: 1
// is_bigendian: False
// point_step: 32 // Length of a point in bytes 一个点占的比特数 
// row_step: 27282496 // Length of a row in bytes 一行的长度占用的比特数
// data: [ .......................................................... ] //  Actual point data, size is (row_step*height)
// is_dense: True // 没有非法数据点
//PointCloud2的data是序列化后的数据，通常看不到物理意义，需要解析
void PointCloud2::PointCallback(const sensor_msgs::PointCloud2::ConstPtr &point_msg)
{
    ROS_INFO_STREAM(
        "header: " << point_msg->header.seq << 
        ", time stamp: " << point_msg->header.stamp << 
        ", frame_id: " << point_msg->header.frame_id << 
        ", height: " << point_msg->height << 
        ", wdith: " << point_msg->wdith << 
        ", pint_step: " << point_msg->point_step << 
        ", row_step: " << point_msg->row_step);

    // 第5个点的欧式坐标为

	sensor_msgs::PointCloud out_pointcloud;
	sensor_msgs::convertPointCloud2ToPointCloud(point_msg, out_pointcloud);
	// for (int i=0; i<out_pointcloud.points.size(); i++) {
		// cout << out_pointcloud.points[i].x << ", " << out_pointcloud.points[i].y << ", " << out_pointcloud.points[i].z << endl;
	// }
	cout << "------" << endl;

    ROS_INFO_STREAM(
        ",x = " << out_pointcloud.points[4].x << ", y = " << out_pointcloud.points[4].y<<",z="<<out_pointcloud.points[4].z;
    );

    // 通过ranges中数据的个数进行雷达数据的遍历
    // for (int i = 0; i < scan_msg->ranges.size(); i++)
    // {

    // }

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_lidar_point_node"); // 节点的名字
    PointCloud2 PointCloud2;

    ros::spin();    // 程序执行到此处时开始进行等待，每次订阅的消息到来都会执行一次ScanCallback()
    return 0;
}