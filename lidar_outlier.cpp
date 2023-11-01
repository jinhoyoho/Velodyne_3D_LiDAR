#include "lidar_header.h"

// Removing outliers using a StatisticalOutlierRemoval filter
// http://pointclouds.org/documentation/tutorials/statistical_outlier.php#statistical-outlier-removal

// 수정할 변수
// setMeanK: 점의 개수
// setStddevMulThresh: outlier 거리 정보

ros::Publisher pub;

void outlier_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PCLPointCloud2* cloud_intermediate = new pcl::PCLPointCloud2;
    pcl::PointCloud<pcl::PointXYZI> cloud;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

    pcl_conversions::toPCL(*input, *cloud_intermediate); // input을 cloud에 저장
    
    // pointer로 옮기기
    pcl::fromPCLPointCloud2(*cloud_intermediate, cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_p = cloud.makeShared();

    // 1. Statistical Outlier Removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud (cloud_p);            //입력 
    sor.setMeanK (100);                    //분석시 고려한 이웃 점 수
    sor.setStddevMulThresh (0.1);         //Outlier로 처리할 거리 정보 
    sor.filter (*cloud_filtered);         // 필터 적용 

    // // 2. Radius Outlier removal
    // 값이 나오기는 하나 이상하게 나와서 포기 -> 나중에 성공하길 바랍니다.,..
    // pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
    // outrem.setInputCloud(cloud_p);    //입력 
    // outrem.setRadiusSearch(0.1);    //탐색 범위 0.01
    // outrem.setMinNeighborsInRadius (10); //최소 보유 포인트 수 10개 
    // outrem.filter (*cloud_filtered);  // 필터 적용 

    pub.publish(*cloud_filtered);
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "lidar_outlier");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("lidar_roi", 1, outlier_callback);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("lidar_outlier",1);

    std::cout << "outlier complete" << std::endl;

    ros::spin();
}