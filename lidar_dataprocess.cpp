#include "lidar_header.h"  //header.h 참조

using namespace std;

// hyperparameter
// 1. roi: setFilterLimits 범위
// 2. voxel: setLeafSize 크기
// 3. ransac: setMaxIterations 반복횟수 setDistanceThreshold 거리 임계값
// 4. dbscan: 아래의 변수들, tmp.z의 범위
// rviz의 frame_id는 velodyne

/*dbscan clustering 사용을 위한 변수*/
double epsilon = 0.3; //Core Point 기준 주변 탐색 반경 
size_t minPoints = 3; //Core Point 기준 필요 인접점 최소 개수

std::vector<std::vector<uint8_t>> COLORS = {
    {255, 0, 0},
    {0, 255, 0},
    {0, 0, 255},
    {255, 255, 0},
    {255, 0, 255},
    {0, 255, 255},
    {128, 255, 0},
    {255, 128, 0},
    {128, 0, 255},
    {255, 0, 128},
    {0, 128, 255},
    {0, 255, 128},
    {128, 255, 255},
    {255, 128, 255},
    {255, 255, 128},
    {60, 180, 0},
    {180, 60, 0},
    {0, 60, 180},
    {0, 180, 60},
    {60, 0, 180},
    {180, 0, 60},
    {255, 0, 0},
    {0, 255, 0},
    {0, 0, 255},
    {255, 255, 0},
    {255, 0, 255},
    {0, 255, 255},
    {128, 255, 0},
    {255, 128, 0},
    {128, 0, 255},
};


pcl::PassThrough<pcl::PointXYZI> pass; // roi 설정을 위한 pass

ros::Publisher pub; // dbscan한 결과를 pub

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

        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud (cloud_out);
        extract.setIndices (inliers);
        extract.setNegative (true); //false
        extract.filter (*inlierPoints_neg);

        // 4. dbscan

        arma::mat data(3, inlierPoints_neg->points.size());

        for (size_t i = 0; i < inlierPoints_neg->points.size(); ++i)
        {
            data(0, i) = inlierPoints_neg->points[i].x;
            data(1, i) = inlierPoints_neg->points[i].y;
            data(2, i) = inlierPoints_neg->points[i].z;
        }

        mlpack::dbscan::DBSCAN<> dbscan(epsilon, minPoints);

        arma::Row<size_t> assignments;
        dbscan.Cluster(data, assignments);

        // DBSCAN 결과를 Colored PointCloud로 변환
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (size_t i = 0; i < assignments.n_elem; ++i)
        {
            if (inlierPoints_neg->points[i].z < -0.2) // -0.2 보다 작은것들은 넘김
            {
                continue;
            }
            pcl::PointXYZRGB point;
            point.x = inlierPoints_neg->points[i].x;
            point.y = inlierPoints_neg->points[i].y;
            point.z = inlierPoints_neg->points[i].z;


            // 클러스터에 따라 색상 설정
            size_t color_index = assignments[i] % COLORS.size();  // 클러스터 수가 색상 배열 크기보다 큰 경우를 대비하여 모듈러 연산 사용
            point.r = COLORS[color_index][0];
            point.g = COLORS[color_index][1];
            point.b = COLORS[color_index][2];

            result_cloud->points.push_back(point);
        }

        sensor_msgs::PointCloud2 cluster;
        pcl::toROSMsg(*result_cloud, cluster);

        // pcl::toROSMsg(cloud_p, cluster);
        cluster.header.frame_id = "macaron";
        cluster.header.stamp = input->header.stamp;

        pub.publish(cluster); // 값을 전달하는 것
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

    std::cout << "process complete" << std::endl;

    ros::spin();
}