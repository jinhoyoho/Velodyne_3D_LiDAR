#include "lidar_header.h"  //header.h 참조
#include "dbscan.h"  // dbscan.h 참조

// 출처: https://cs-kookmin-club.tistory.com/144 (국민대 FOSCAR)

using namespace std;

/*dbscan clustering 사용을 위한 변수*/
int minPoints = 3;          //Core Point 기준 필요 인접점 최소 개수
double epsilon = 0.3;      //Core Point 기준 주변 탐색 반경 

//clustering 사이즈 설정해주어서 사이즈에 해당하는 clustring만 보여줌 - clustering 사이즈에 따라서 인식할 수 있는 물체가 달라짐 
// min clustering과 maxclustering size를 이용하여 미션시 라바콘과 실제 차량의 차이를 구별 할 수 있지 않을까 생각됨.
int minClusterSize = 1;     //Cluster 최소 사이즈
int maxClusterSize = 10000;  //Cluster 최대 사이즈

//ROI를 이용하여 걸러내진 못한 물체들을 BoundingBOX변수를 이용하여 한번 더 filtering 할 수 있음.
double xMinBoundingBox = -3;
double xMaxBoundingBox = 3;
double yMinBoundingBox = -3;
double yMaxBoundingBox = 3;
double zMinBoundingBox = -3;
double zMaxBoundingBox = 3; // BoundingBox 크기 범위 지정 변수

ros::Publisher pub;
ros::Publisher boundingBoxPub; //Bounding Box Visualization Publisher

void dbscan_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    //ROS message 변환
    //PointXYZI가 아닌 PointXYZ로 선언하는 이유 -> 각각의 Cluster를 다른 색으로 표현해주기 위해서. Clustering 이후 각각 구별되는 intensity value를 넣어줄 예정.
    //PointXYZI는 intensity정보를 가지고 있고, 이것은 반사강도를 나타내는데 크게 중요하지 않기 때문에 PointXYZ를 사용함.
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);//스마트 포인터를 선언하고, 이 포인터를 통해 새로운 Point Cloud를 생성
    pcl::fromROSMsg(*input, *cloud); // ROS에서 받아온 Point Cloud 메시지를 PCL의 Point Cloud 형식으로 변환

    //KD-Tree
    //이 포인터를 통해 KD-Tree 검색 방법을 사용할 수 있도록 하는 객체를 생성합니다
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    //Point Cloud(cloud)가 비어있지 않은 경우에 한해 KD-Tree 검색 방법에서 사용할 입력 Point Cloud를 설정
    if (cloud->size() > 0) 
    {
        tree->setInputCloud(cloud);
    }
    //Segmentation
    //클러스터의 index 정보를 저장할 vector를 선언합니다.
    vector<pcl::PointIndices> cluster_indices;

    DBSCANKdtreeCluster<pcl::PointXYZI> dc;
    dc.setCorePointMinPts(minPoints);   //Set minimum number of neighbor points
    dc.setClusterTolerance(epsilon); //Set Epsilon 
    dc.setMinClusterSize(minClusterSize);
    dc.setMaxClusterSize(maxClusterSize);
    dc.setSearchMethod(tree); // kdtree
    dc.setInputCloud(cloud); // point_cloud xyz
    dc.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZI> totalcloud_clustered; // 클러스터 하나의 point cloud
    int cluster_id = 0;
    visualization_msgs::Marker boundingBox;  // xyz를 boundingbox 처리
    visualization_msgs::MarkerArray boundingBoxArray; // boundingbox들을 array로 만들어서 한번에 여러개의 boundingbox를 보여줌
    
    for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++, cluster_id++) 
    {
        pcl::PointCloud<pcl::PointXYZI> eachcloud_clustered; // 클러스터 하나의 point cloud
        float cluster_counts = cluster_indices.size();

        //각 Cluster내 각 Point 접근
        for(vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) 
        {
            pcl::PointXYZI tmp;
            tmp.x = cloud->points[*pit].x;
            tmp.y = cloud->points[*pit].y;
            tmp.z = cloud->points[*pit].z;

            if (tmp.z < -0.3) // dbscan의 roi를 설정 -> 제거되지 않은 지면 없애기
            {
                continue;
            }
            
            tmp.intensity = cluster_id; // 클러스터별로 색을 다르게 하기 위해 tmp.intensity 값을 cluster_id % 8로 설정
            eachcloud_clustered.push_back(tmp); // 클러스터 하나의 point cloud
            totalcloud_clustered.push_back(tmp);
        }

        // dc.check_z(totalcloud_clustered, eachcloud_clustered);


        //minPoint와 maxPoint 받아오기
        // 각 클러스터의 bounding box의 크기를 계산하기 위해, pcl::getMinMax3D 함수를 통해 eachcloud_clustered의 최소값과 최대값을 구합니다.
        pcl::PointXYZI minPoint, maxPoint;
        pcl::getMinMax3D(eachcloud_clustered, minPoint, maxPoint);

        float x_len = abs(maxPoint.x - minPoint.x);   //직육면체 x 모서리 크기
        float y_len = abs(maxPoint.y - minPoint.y);   //직육면체 y 모서리 크기
        float z_len = abs(maxPoint.z - minPoint.z);   //직육면체 z 모서리 크기 
        float volume = x_len * y_len * z_len;         //직육면체 부피

        float center_x = (minPoint.x + maxPoint.x)/2; //직육면체 중심 x 좌표
        float center_y = (minPoint.y + maxPoint.y)/2; //직육면체 중심 y 좌표
        float center_z = (minPoint.z + maxPoint.z)/2; //직육면체 중심 z 좌표 

        // 라이다 센서의 위치는 0,0,0이기 때문에 이를 바탕으로 거리정보를 구할 수 있음
        float distance = sqrt(center_x * center_x + center_y * center_y); //장애물 <-> 차량 거리

          if ( (xMinBoundingBox < x_len && x_len < xMaxBoundingBox) && (yMinBoundingBox < y_len && y_len < yMaxBoundingBox) && (zMinBoundingBox < z_len && z_len < zMaxBoundingBox) ) 
          {
                boundingBox.header.frame_id = "velodyne";
                boundingBox.header.stamp = ros::Time();
                boundingBox.ns = cluster_counts; //ns = namespace
                boundingBox.id = cluster_id; 
                boundingBox.type = visualization_msgs::Marker::CUBE; //직육면체로 표시
                boundingBox.action = visualization_msgs::Marker::ADD;
                
                // 각 좌표의 중심
                boundingBox.pose.position.x = center_x; 
                boundingBox.pose.position.y = center_y;
                boundingBox.pose.position.z = center_z;
                
                boundingBox.pose.orientation.x = 0.0;
                boundingBox.pose.orientation.y = 0.0;
                boundingBox.pose.orientation.z = 0.0;
                boundingBox.pose.orientation.w = 1.0; // w 값이 1.0이면, 바운딩 박스의 회전이 없음을 의미. 
                //즉, bounding box가 원점에 대해 정렬되어 있어서 회전하지 않은 상태입니다. 이는 bounding box의 모든 면이 축에 평행하게 정렬되어 있을 때 발생할 수 있습니다.
                
                // 길이 
                boundingBox.scale.x = x_len;
                boundingBox.scale.y = y_len;
                boundingBox.scale.z = z_len;
                
                // bounding box 색깔 설정
                boundingBox.color.a = 0.5; //직육면체 투명도, a = alpha
                boundingBox.color.r = 1.0; //직육면체 색상 RGB값
                boundingBox.color.g = 1.0;
                boundingBox.color.b = 1.0;
                
                // boundingbox 지속 시간 및 boundingBoxArray에 boundingbox추가
                // boundingbox지속시간을 0.1로 준 이유는 cluster한개만 보이는 것은 필요없기 때문에 빠르게 사라지게 하기 위해서 시간을 짧게 설정
                // emplace_back은 push_back과 같속
                boundingBox.lifetime = ros::Duration(0.1); //box 지속시간
                boundingBoxArray.markers.emplace_back(boundingBox);
            }

        cluster_id++; //intensity 증가
    }

    //Convert To ROS data type
    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(totalcloud_clustered, cloud_p);

    sensor_msgs::PointCloud2 cluster;
    pcl_conversions::moveFromPCL(cloud_p, cluster);
    // pcl::toROSMsg(cloud_p, cluster);
    cluster.header.frame_id = "velodyne";

    pub.publish(cluster); // 값을 전달하는 것
    boundingBoxPub.publish(boundingBoxArray);
    
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "lidar_dbscan");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("lidar_ransac", 1, dbscan_callback);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("lidar_dbscan",1);
    boundingBoxPub = nh.advertise<visualization_msgs::MarkerArray>("/boundingBox", 0.001);   

    std::cout << "dbscan complete" << std::endl;

    ros::spin();
}