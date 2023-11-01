#include "lidar_header.h"  //header.h 참조
#include "dbscan.h"  // dbscan.h 참조

using namespace std;

// hyperparameter
// 1. roi: setFilterLimits 범위
// 2. voxel: setLeafSize 크기
// 3. ransac: setMaxIterations 반복횟수 setDistanceThreshold 거리 임계값
// 4. dbscan: 아래의 변수들, tmp.z의 범위
// rviz의 frame_id는 velodyne


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

pcl::PassThrough<pcl::PointXYZI> pass; // roi 설정을 위한 pass

ros::Publisher pub; // dbscan한 결과를 pub
ros::Publisher boundingBoxPub; // dbscan bounding box rviz를 위한 pub

std::string state = "static_obstacle"; // crusing mode로 시작

void state_callback(const std_msgs::String::ConstPtr& msg)
{
    state = msg->data; // state 저장
    std::cout << state << std::endl;
}

void cloud_callBack(const sensor_msgs::PointCloud2ConstPtr& input)
{
    if (state !="cruising")
    {   
        // container for original and filtered data
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

        // ros msg를 pointcloud로 변경
        pcl::fromROSMsg(*input, *cloud);

        // 1. roi 설정
        // lidar 좌표계 기준 x, y, z 설정 가능
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
        //pcl::pointcloud를 pcl::pointcloud<XYZI>::ptr로 변경

        if (state == "static_obstacle")
        {
            // Apply Passthrough Filter
            pass.setInputCloud (cloud); // raw data 입력
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (-2, 2);   // 상하거리
            //pass.setFilterLimitsNegative (true);
            pass.filter (*cloud_out);

            // // Apply Passthrough Filter
            pass.setFilterFieldName ("x");
            pass.setFilterLimits (0, 60);  // 앞뒤거리
            pass.setInputCloud (cloud_out);
            pass.filter (*cloud_out);

            pass.setFilterFieldName ("y");
            pass.setFilterLimits (-8.0, 8.0); //좌우거리
            // pass.setFilterLimitsNegative (true);
            pass.setInputCloud (cloud_out);
            pass.filter (*cloud_out);
        }
        else if (state == "rotary")
        {
            // Apply Passthrough Filter
            pass.setInputCloud (cloud);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (-2, 2);   // 상하거리
            //pass.setFilterLimitsNegative (true);
            pass.filter (*cloud_out);

            // // Apply Passthrough Filter
            pass.setFilterFieldName ("x");
            pass.setFilterLimits (0, 60);  // 앞뒤거리
            pass.setInputCloud (cloud_out);
            pass.filter (*cloud_out);

            pass.setFilterFieldName ("y");
            pass.setFilterLimits (-8.0, 8.0); //좌우거리
            // pass.setFilterLimitsNegative (true);
            pass.setInputCloud (cloud_out);
            pass.filter (*cloud_out);
        }
        else // end
        {
            // Apply Passthrough Filter
            pass.setInputCloud (cloud);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (-5, 13);   // 상하거리
            //pass.setFilterLimitsNegative (true);
            pass.filter (*cloud_out);

            // // Apply Passthrough Filter
            pass.setFilterFieldName ("x");
            pass.setFilterLimits (0, 20);  // 앞뒤거리
            pass.setInputCloud (cloud_out);
            pass.filter (*cloud_out);

            pass.setFilterFieldName ("y");
            pass.setFilterLimits (-10, 10); //좌우거리
            // pass.setFilterLimitsNegative (true);
            pass.setInputCloud (cloud_out);
            pass.filter (*cloud_out);
        }
        
        // 2. voxel 처리
        // pcl 의 VoxelGrid 타입
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        pcl::PCLPointCloud2::Ptr cloud_voxel(new pcl::PCLPointCloud2);

        // voxel은 <XYZI>가 안 들어간다.
        pcl::PCLPointCloud2::Ptr cloud_out_PCL2(new pcl::PCLPointCloud2);

        pcl::toPCLPointCloud2(*cloud_out, *cloud_out_PCL2);
        
        sor.setInputCloud (cloud_out_PCL2);
        // 샘플링 하는 방법 이거 너무 작게 하면 샘플링 에러 메세지 뜸 고것을 주의 하자
        //leaf size  1cm 격자의 x, y, z 크기

        if (state == "static_obstacle")
        {
            sor.setLeafSize (0.2f, 0.2f, 0.2f); // voxel 크기 설정
        }
        else if (state == "rotary")
        {
            sor.setLeafSize (0.2f, 0.2f, 0.2f); // voxel 크기 설정
        }
        else
        {
            sor.setLeafSize (0.1f, 0.1f, 0.1f); // voxel 크기 설정
        }

        sor.filter(*cloud_voxel);

        // Pointcloud2를 <XYZI>로 변경
        pcl::fromPCLPointCloud2(*cloud_voxel, *cloud_out);

        // 3. ransac
        pcl::PointCloud<pcl::PointXYZI>::Ptr inlierPoints (new pcl::PointCloud<pcl::PointXYZI>),
                                            inlierPoints_neg (new pcl::PointCloud<pcl::PointXYZI>);

        // Object for storing the plane model coefficients.
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());


        // 오프젝트 생성 Create the segmentation object.
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients (true);       //(옵션) // Enable model coefficient refinement (optional).
        seg.setModelType (pcl::SACMODEL_PLANE);    //적용 모델  // Configure the object to look for a plane.
        seg.setMethodType (pcl::SAC_RANSAC);       //적용 방법   // Use RANSAC method.
        seg.setMaxIterations (2000);               //최대 실행 수
        seg.setDistanceThreshold (0.08);          //inlier로 처리할 거리 정보   // Set the maximum allowed distance to the model.
        //seg.setRadiusLimits(0, 0.1);     // cylinder경우, Set minimum and maximum radii of the cylinder.
        seg.setInputCloud (cloud_out);                //입력 
        seg.segment (*inliers, *coefficients);    //세그멘테이션 적용 


        pcl::copyPointCloud<pcl::PointXYZI>(*cloud_out, *inliers, *inlierPoints);

        //[옵션]] 바닥 제거 결과 얻기 
        //Extracting indices from a PointCloud
        //http://pointclouds.org/documentation/tutorials/extract_indices.php
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud (cloud_out);
        extract.setIndices (inliers);
        extract.setNegative (true); //false
        extract.filter (*inlierPoints_neg);

    
        // 4. dbscan

        //ROS message 변환
        //PointXYZI가 아닌 PointXYZ로 선언하는 이유 -> 각각의 Cluster를 다른 색으로 표현해주기 위해서. Clustering 이후 각각 구별되는 intensity value를 넣어줄 예정.
        //PointXYZI는 intensity정보를 가지고 있고, 이것은 반사강도를 나타내는데 크게 중요하지 않기 때문에 PointXYZ를 사용함.

        //KD-Tree
        //이 포인터를 통해 KD-Tree 검색 방법을 사용할 수 있도록 하는 객체를 생성합니다
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        //Point Cloud(cloud)가 비어있지 않은 경우에 한해 KD-Tree 검색 방법에서 사용할 입력 Point Cloud를 설정
        if (inlierPoints_neg->size() > 0) 
        {
            tree->setInputCloud(inlierPoints_neg);
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
        dc.setInputCloud(inlierPoints_neg); // point_cloud xyz
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
                tmp.x = inlierPoints_neg->points[*pit].x;
                tmp.y = inlierPoints_neg->points[*pit].y;
                tmp.z = inlierPoints_neg->points[*pit].z;

                if (tmp.z < -0.1) // dbscan의 roi를 설정 -> 제거되지 않은 지면 없애기
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
                    boundingBox.header.frame_id = "macaron";
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
        cluster.header.frame_id = "macaron";
        cluster.header.stamp = input->header.stamp;

        pub.publish(cluster); // 값을 전달하는 것
        boundingBoxPub.publish(boundingBoxArray);
    }
    else
    {
        return; // 종료
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_process");
    ros::NodeHandle nh;

    std::cout << state << std::endl;


    ros::Subscriber sub_state = nh.subscribe("/lidar_mode", 1, state_callback);
    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, cloud_callBack);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("lidar_data", 1);
    boundingBoxPub = nh.advertise<visualization_msgs::MarkerArray>("/boundingBox", 0.001);  

    std::cout << "process complete" << std::endl;

    ros::spin();
}
