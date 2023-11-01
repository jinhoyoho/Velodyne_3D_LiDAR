#include "lidar_header.h"

// double ROI_theta(double x, double y);
// using namespace std;

ros::Publisher pub;
pcl::PassThrough<pcl::PointXYZI> pass;

void roi_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PCLPointCloud2* cloud_intermediate = new pcl::PCLPointCloud2;
    pcl::PointCloud<pcl::PointXYZI> cloud;

    // Convert to PCL data type
    pcl_conversions::toPCL(*input, *cloud_intermediate);

    // Convert PCL::PointCloud2 to PCL::PointCloud<PointXYZI>
    pcl::fromPCLPointCloud2(*cloud_intermediate, cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_p = cloud.makeShared();

    // Apply Passthrough Filter
    pass.setInputCloud (cloud_p);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-2, 1);   // 상하거리
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_p);

    // // Apply Passthrough Filter
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0, 55);  // 앞뒤거리
    pass.setInputCloud (cloud_p);
    pass.filter (*cloud_p);

    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-5.0, 5.0); //좌우거리
    // pass.setFilterLimitsNegative (true);
    pass.setInputCloud (cloud_p);
    pass.filter (*cloud_p);

    pub.publish(*cloud_p);
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "lidar_roi");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("lidar_voxel", 1, roi_callback);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("lidar_roi",1);

    std::cout << "roi complete" << std::endl;

    ros::spin();
}


// 수학적으로 계산한 roi, 라이다의 각도만 사용

// double ROI_theta(double x, double y)
// {
//     double r;
//     double theta;

//     r = sqrt((x*x)+(y*y));
//     theta = acos(x/r)*180/M_PI;
//     return theta;
// }

// void roi_callback(const sensor_msgs::PointCloud2ConstPtr& input)
// {
// 	//1. Msg to pointcloud
//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
//     pcl::fromROSMsg(*input,*cloud); // sensor msg를 pcl로 변경

//     double theta_r = 0; // 회전 각도

//     //2. 회전변환행렬
//     Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
//     transform_1 (0,0) = std::cos (theta_r);
//     transform_1 (0,1) = -sin(theta_r);
//     transform_1 (1,0) = sin (theta_r);
//     transform_1 (1,1) = std::cos (theta_r);
//     // Executing the transformation
  
//     pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
//     pcl::transformPointCloud (*cloud, *transformed_cloud, transform_1); 
//     //transform_1 의형식으로 cloud 를 transformed_cloud로 변환
  
//     pcl::PCLPointCloud2 cloud_p;
//     pcl::toPCLPointCloud2(*transformed_cloud, cloud_p);
//     sensor_msgs::PointCloud2 output;
//     pcl_conversions::fromPCL(cloud_p, output);
//     output.header.frame_id = "velodyne";
//     // pub1.publish(output);
    
// 	//3. ROI 설정
//     pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
//     pcl::fromROSMsg(output,laserCloudIn); 
//     pcl::PCLPointCloud2 cloud_ROI;

    
//     for(unsigned int j=0; j<laserCloudIn.points.size(); j++)
// 	{
//         if(ROI_theta(laserCloudIn.points[j].y , laserCloudIn.points[j].x) < 45) // 45도와
//         {
//             laserCloudIn.points[j].x = 0;
//             laserCloudIn.points[j].y = 0;
//             laserCloudIn.points[j].z = 0;
//         }
//         if(ROI_theta(laserCloudIn.points[j].y , laserCloudIn.points[j].x) > 135) // 135도 사이 값
//         {
//             laserCloudIn.points[j].x = 0;
//             laserCloudIn.points[j].y = 0;
//             laserCloudIn.points[j].z = 0;
//         }
//         if(laserCloudIn.points[j].x < 0)
//         {
//             laserCloudIn.points[j].x = 0;
//             laserCloudIn.points[j].y = 0;
//             laserCloudIn.points[j].z = 0;
//         }
//     }

//     pcl::toPCLPointCloud2(laserCloudIn, cloud_ROI);
//     sensor_msgs::PointCloud2 output_ROI; //출력할 방식인 PC2 선정 및 이름 output_ROI 정의
//     pcl_conversions::fromPCL(cloud_ROI, output_ROI);
//     output_ROI.header.frame_id = "velodyne";
//     pub.publish(output_ROI);
// }


//Filtering a PointCloud using a PassThrough filter
//http://pointclouds.org/documentation/tutorials/passthrough.php#passthrough
