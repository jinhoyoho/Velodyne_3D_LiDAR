#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <mlpack.hpp>
#include <mlpack/methods/dbscan/dbscan.hpp>
#include <armadillo>

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  arma::mat data(3, cloud->points.size());
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    data(0, i) = cloud->points[i].x;
    data(1, i) = cloud->points[i].y;
    data(2, i) = cloud->points[i].z;
  }

  double epsilon = 1.0;
  size_t minPoints = 5;
  mlpack::dbscan::DBSCAN<> dbscan(epsilon, minPoints);

  arma::Row<size_t> assignments;
  dbscan.Cluster(data, assignments);

  // DBSCAN 결과를 Colored PointCloud로 변환
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (size_t i = 0; i < assignments.n_elem; ++i)
  {
    pcl::PointXYZRGB point;
    point.x = cloud->points[i].x;
    point.y = cloud->points[i].y;
    point.z = cloud->points[i].z;

    // 클러스터에 따라 색상 설정
    if (assignments[i] == 0)
    {
      point.r = 255; point.g = 0; point.b = 0; // Red
    }
    else if (assignments[i] == 1)
    {
      point.r = 0; point.g = 255; point.b = 0; // Green
    }
    else if (assignments[i] == 2)
    {
      point.r = 0; point.g = 0; point.b = 255; // Blue
    }
    // ... 필요한 만큼 색상 추가

    result_cloud->points.push_back(point);
  }

  // pcl::PointCloud를 sensor_msgs/PointCloud2로 변환
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*result_cloud, output);

  output.header.frame_id = "velodyne";

  // 변환된 메시지 publish
  pub.publish(output);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_dbscan");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("lidar_ransac", 1, cloud_cb);
  pub = nh.advertise<sensor_msgs::PointCloud2>("dbscan_result", 1);

  ros::spin();

  return 0;
}
