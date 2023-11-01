#include "lidar_header.h"

ros::Publisher pub;

void cloud_callBack(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // container for original and filtered data
    pcl::PCLPointCloud2 * cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud); // pointer for dynamic cloud
    pcl::PCLPointCloud2 cloud_filtered;

    //conversion ROS sensor_msg/PointCloud2 -> pcl lib의 PointCloud2 type 
    pcl_conversions::toPCL(*input, *cloud);
    
    // pcl 의 VoxelGrid 타입
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    // 샘플링 하는 방법 이거 너무 작게 하면 샘플링 에러 메세지 뜸 고것을 주의 하자
    //leaf size  1cm 격자의 x, y, z 크기
    sor.setLeafSize (0.1f, 0.1f, 0.1f);
    sor.filter(cloud_filtered);

    sensor_msgs::PointCloud2 output;
    //pcl lib PointCLoud2 type -> ROS sensor_msgs data
    pcl_conversions::moveFromPCL(cloud_filtered, output);
    pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_voxel");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, cloud_callBack);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("lidar_voxel", 1);

    std::cout << "voxel complete" << std::endl;

    ros::spin();
}