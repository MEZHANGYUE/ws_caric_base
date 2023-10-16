#include <ros/ros.h>
#include <thread>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <cmath>
#include <string>
#include <vector>
#include <chrono>
#include <cstdlib>
using std::vector;
using namespace std::chrono_literals;
float Fov_Really_size = 0;  // 视点的最大索引数值
double fov_horizontal_deg = 60.0;  // 水平FOV（度）
double fov_vertical_deg = 45.0;  // 垂直FOV（度）
double fov_distance =1.0;
double fov_half_width = std::tan(fov_horizontal_deg / 2.0 * M_PI / 180.0) * fov_distance;  // 计算FOV的边界点
double fov_half_height = std::tan(fov_vertical_deg / 2.0 * M_PI / 180.0) * fov_distance;
std::set<int> FovPointIndices;  //FOV索引
std::vector<int> AllpointIndices;
std::vector<int> SelectedPointIndices;

struct FoVInterest {
public:
    float x;
    float y;
    float z;
    float pitch;
    float yaw;
    float roll;
};// 装视点的结构
// void init() {
//     x = 0.0;
//     y = 0.0;
//     z = 0.0;
//     pitch = 0.0;
//     yaw = 0.0;
//     roll = 0.0;
// }
FoVInterest interest0;
vector<FoVInterest> fov_interests;

void show_fov(pcl::visualization::PCLVisualizer::Ptr views, vector<FoVInterest>& vec)
{
    int i=0; int skipCount = 0;
    for(vector<FoVInterest>::iterator it = vec.begin(); it != vec.end(); ++it, ++i)
    {
        // if (skipCount < 15) 
        // {
        //     skipCount++;
        //     continue;
        // }
        // else skipCount = 0;
        //输入数据
        float x = it->x;
        float y = it->y;
        float z = it->z;
        float pitch = it->pitch;
        float yaw = it->yaw;
        float roll = it->roll;
        double fov_distance =1.0;

        // 创建一个变换矩阵来表示位姿
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << x, y, z; // 设置平移
        transform.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitY())); // 设置偏航
        transform.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitX())); // 设置俯仰
        transform.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitZ())); // 设置横滚

        Eigen::Vector3f fov_center(0.0, 0.0, fov_distance); // FOV中心点坐标
        Eigen::Vector3f fov_top_left(-fov_half_width, fov_half_height, fov_distance);
        Eigen::Vector3f fov_top_right(fov_half_width, fov_half_height, fov_distance);
        Eigen::Vector3f fov_bottom_left(-fov_half_width, -fov_half_height, fov_distance);
        Eigen::Vector3f fov_bottom_right(fov_half_width, -fov_half_height, fov_distance);
        Eigen::Vector3f fov_base(x, y, z);

        // 应用位姿变换到FOV边界点
        fov_center = transform * fov_center;
        fov_top_left = transform * fov_top_left;
        fov_top_right = transform * fov_top_right;
        fov_bottom_left = transform * fov_bottom_left;
        fov_bottom_right = transform * fov_bottom_right;
        
        // 绘制FOV的边界线
        std::string temp_str1="fova";temp_str1+=std::to_string(i);std::string temp_str2="fovb";temp_str2+=std::to_string(i);
        std::string temp_str3="fovc";temp_str3+=std::to_string(i);std::string temp_str4="fovd";temp_str4+=std::to_string(i);
        std::string temp_str5="fove";temp_str5+=std::to_string(i);std::string temp_str6="fovf";temp_str6+=std::to_string(i);
        std::string temp_str7="fovg";temp_str7+=std::to_string(i);std::string temp_str8="fovh";temp_str8+=std::to_string(i);
        views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_top_left.x(), fov_top_left.y(), fov_top_left.z()), pcl::PointXYZ(fov_top_right.x(), fov_top_right.y(), fov_top_right.z()), 1.0, 0.0, 0.0, temp_str1);
        views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_top_right.x(), fov_top_right.y(), fov_top_right.z()), pcl::PointXYZ(fov_bottom_right.x(), fov_bottom_right.y(), fov_bottom_right.z()), 1.0, 0.0, 0.0, temp_str2);
        views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_bottom_right.x(), fov_bottom_right.y(), fov_bottom_right.z()), pcl::PointXYZ(fov_bottom_left.x(), fov_bottom_left.y(), fov_bottom_left.z()), 1.0, 0.0, 0.0, temp_str3);
        views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_bottom_left.x(), fov_bottom_left.y(), fov_bottom_left.z()), pcl::PointXYZ(fov_top_left.x(), fov_top_left.y(), fov_top_left.z()), 1.0, 0.0, 0.0, temp_str4);
        views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_base.x(), fov_base.y(), fov_base.z()), pcl::PointXYZ(fov_top_left.x(), fov_top_left.y(), fov_top_left.z()), 1.0, 0.0, 0.0, temp_str5);
        views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_base.x(), fov_base.y(), fov_base.z()), pcl::PointXYZ(fov_top_right.x(), fov_top_right.y(), fov_top_right.z()), 1.0, 0.0, 0.0, temp_str6);
        views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_base.x(), fov_base.y(), fov_base.z()), pcl::PointXYZ(fov_bottom_left.x(), fov_bottom_left.y(), fov_bottom_left.z()), 1.0, 0.0, 0.0, temp_str7);
        views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_base.x(), fov_base.y(), fov_base.z()), pcl::PointXYZ(fov_bottom_right.x(), fov_bottom_right.y(), fov_bottom_right.z()), 1.0, 0.0, 0.0, temp_str8);
    }
}

void test_Fov(pcl::visualization::PCLVisualizer::Ptr views, pcl::PointCloud<pcl::PointXYZ>::Ptr clouds, pcl::PointCloud<pcl::Normal>::Ptr normals, vector<FoVInterest>& vec)
{
    int num =304;
    //输入数据
    float x=vec[num].x;
    float y=vec[num].y;
    float z=vec[num].z;
    float pitch=vec[num].pitch;
    float yaw=vec[num].yaw;
    float roll=vec[num].roll;
    std::cout << "测试的 俯仰 偏航: (" << pitch*(180/M_PI)  << ", " << yaw*(180/M_PI) << ")" << std::endl;

    pcl::Normal normal = normals->points[num];
    double nx = normal.normal_x;
    double ny = normal.normal_y;
    double nz = normal.normal_z;
    // 输出法向量信息
    std::cout << "Normal: (" << nx << ", " << ny << ", " << nz << ")" << std::endl;

    // 创建一个变换矩阵来表示位姿
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << x, y, z; // 设置平移        
    transform.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitY())); // 设置偏航
    transform.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitX())); // 设置俯仰
    transform.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitZ())); // 设置横滚

    Eigen::Vector3f fov_center(0.0, 0.0, fov_distance); // FOV中心点坐标
    Eigen::Vector3f fov_top_left(-fov_half_width, fov_half_height, fov_distance);
    Eigen::Vector3f fov_top_right(fov_half_width, fov_half_height, fov_distance);
    Eigen::Vector3f fov_bottom_left(-fov_half_width, -fov_half_height, fov_distance);
    Eigen::Vector3f fov_bottom_right(fov_half_width, -fov_half_height, fov_distance);
    Eigen::Vector3f fov_base(x,y,z);
    
    // 未变化前
    views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_top_left.x(), fov_top_left.y(), fov_top_left.z()), pcl::PointXYZ(fov_top_right.x(), fov_top_right.y(), fov_top_right.z()), 1.0, 0.0, 0.0,"testa10");
    views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_top_right.x(), fov_top_right.y(), fov_top_right.z()), pcl::PointXYZ(fov_bottom_right.x(), fov_bottom_right.y(), fov_bottom_right.z()), 1.0, 0.0, 0.0,"testa20");
    views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_bottom_right.x(), fov_bottom_right.y(), fov_bottom_right.z()), pcl::PointXYZ(fov_bottom_left.x(), fov_bottom_left.y(), fov_bottom_left.z()), 1.0, 0.0, 0.0,"testa30");
    views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_bottom_left.x(), fov_bottom_left.y(), fov_bottom_left.z()), pcl::PointXYZ(fov_top_left.x(), fov_top_left.y(), fov_top_left.z()), 1.0, 0.0, 0.0, "testa40");
    Eigen::Vector3f fov_base0(0,0,0);
    views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_base0.x(), fov_base0.y(), fov_base0.z()), pcl::PointXYZ(fov_top_left.x(), fov_top_left.y(), fov_top_left.z()), 1.0, 0.0, 0.0,"testa50");
    views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_base0.x(), fov_base0.y(), fov_base0.z()), pcl::PointXYZ(fov_top_right.x(), fov_top_right.y(), fov_top_right.z()), 1.0, 0.0, 0.0,"testa60");
    views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_base0.x(), fov_base0.y(), fov_base0.z()), pcl::PointXYZ(fov_bottom_left.x(), fov_bottom_left.y(), fov_bottom_left.z()), 1.0, 0.0, 0.0,"testa70");
    views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_base0.x(), fov_base0.y(), fov_base0.z()), pcl::PointXYZ(fov_bottom_right.x(), fov_bottom_right.y(), fov_bottom_right.z()), 1.0, 0.0, 0.0,"testa80");

    // 应用位姿变换到FOV边界点
    fov_center = transform * fov_center;
    fov_top_left = transform * fov_top_left;
    fov_top_right = transform * fov_top_right;
    fov_bottom_left = transform * fov_bottom_left;
    fov_bottom_right = transform * fov_bottom_right;
    
    // 绘制FOV的边界线
    views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_top_left.x(), fov_top_left.y(), fov_top_left.z()), pcl::PointXYZ(fov_top_right.x(), fov_top_right.y(), fov_top_right.z()), 1.0, 0.0, 0.0,"testa1");
    views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_top_right.x(), fov_top_right.y(), fov_top_right.z()), pcl::PointXYZ(fov_bottom_right.x(), fov_bottom_right.y(), fov_bottom_right.z()), 1.0, 0.0, 0.0,"testa2");
    views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_bottom_right.x(), fov_bottom_right.y(), fov_bottom_right.z()), pcl::PointXYZ(fov_bottom_left.x(), fov_bottom_left.y(), fov_bottom_left.z()), 1.0, 0.0, 0.0,"testa3");
    views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_bottom_left.x(), fov_bottom_left.y(), fov_bottom_left.z()), pcl::PointXYZ(fov_top_left.x(), fov_top_left.y(), fov_top_left.z()), 1.0, 0.0, 0.0, "testa4");
    views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_base.x(), fov_base.y(), fov_base.z()), pcl::PointXYZ(fov_top_left.x(), fov_top_left.y(), fov_top_left.z()), 1.0, 0.0, 0.0,"testa5");
    views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_base.x(), fov_base.y(), fov_base.z()), pcl::PointXYZ(fov_top_right.x(), fov_top_right.y(), fov_top_right.z()), 1.0, 0.0, 0.0,"testa6");
    views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_base.x(), fov_base.y(), fov_base.z()), pcl::PointXYZ(fov_bottom_left.x(), fov_bottom_left.y(), fov_bottom_left.z()), 1.0, 0.0, 0.0,"testa7");
    views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_base.x(), fov_base.y(), fov_base.z()), pcl::PointXYZ(fov_bottom_right.x(), fov_bottom_right.y(), fov_bottom_right.z()), 1.0, 0.0, 0.0,"testa8");
    views->addSphere(clouds->points[num], 0.2, 0.5, 0.5, 0.0, "sphere");

    pcl::PointXYZ pclPoint;
    for (size_t i = 0; i < interest0.SelectedPoints.size(); ++i)
    {
        pclPoint.x = interest0.SelectedPoints[i].x();pclPoint.y = interest0.SelectedPoints[i].y();pclPoint.z = interest0.SelectedPoints[i].z();
        std::string temp_sphere1="sphere";temp_sphere1+=std::to_string(i);
        views->addSphere(pclPoint, 0.1, 0.5, 0.5, 0.0, temp_sphere1);
    }   
   
}

void compute_fov(const pcl::Normal& normal, const pcl::PointXYZ& start_point, FoVInterest& interest)
{
    // 计算法向量的方向向量
    Eigen::Vector3d view_direction(normal.normal_x, normal.normal_y, normal.normal_z);
    view_direction.normalize();
    // 计算FOV的距离（可以根据需求调整）
    double fov_distance = 1.0; //用于控制视角和墙壁的距离
    // 计算FOV的位置（xyz坐标）
    Eigen::Vector3d fov_position(start_point.x - fov_distance * view_direction.x(),
                                start_point.y - fov_distance * view_direction.y(),
                                start_point.z - fov_distance * view_direction.z());
    interest.x = fov_position.x();
    interest.y = fov_position.y();
    interest.z = fov_position.z();
    // 计算俯仰角 
    interest.pitch = std::atan2(view_direction.y(), sqrt(view_direction.x()*view_direction.x() + view_direction.z()*view_direction.z()))*(-1); 
    // 俯仰角转换为度
    // 计算偏航角
    interest.yaw = std::atan2(view_direction.x(), view_direction.z());
    // 偏航角转换为度
    // 设置横滚角为0 现在测试打开;
    interest.roll = 0;
} 

void get_fov(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normal, FoVInterest& interest, int& i)
{
    // // 输出法向量和坐标信息
    // for (std::size_t i = 0; i < cloud_normal->size(); ++i) 
    // {}
    pcl::Normal normal = cloud_normal->points[i];
    //访问当前点云信息
    pcl::PointXYZ point = cloud->points[i];
    // 访问法向量的坐标分量
    double nx = normal.normal_x;
    double ny = normal.normal_y;
    double nz = normal.normal_z;
    //计算当前点和法向量对应的视点
    compute_fov(normal, point, interest);
    
    Fov_Really_size = cloud_normal->size();
}

//校正法向量
void correctNormals(Eigen::Vector4f centroids, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal) 
{
    // 校正法向量方向
    for (size_t i = 0; i < normal->size(); ++i) 
    {
        Eigen::Vector3f vector_to_centroid(centroids[0] - cloud->points[i].x,
                                           centroids[1] - cloud->points[i].y,
                                           centroids[2] - cloud->points[i].z);
        // 点积为负值表示法向量指向外部，翻转法向量
        if (normal->points[i].getNormalVector3fMap().dot(vector_to_centroid) < 0) 
        {
            normal->points[i].getNormalVector3fMap() *= -1;
        }
    }
}

//降采样
void downsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& dwcloud)
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(dwcloud);
    sor.setLeafSize(1.5, 1.5, 1.5); // 设置降采样的体素大小
    sor.filter(*dwcloud);
}

//计算法向量
void computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr incloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normal, double radius = 8.0)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(incloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(radius); // 设置搜索半径，根据你的数据调整
    ne.compute(*cloud_normal);
}

//判断是否超出FoV
void selectPointoffov(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normal, vector<int>& indexP) 
{   
    int randnum = indexP[indexP.size()-1];
    get_fov(cloud, cloud_normal, interest0, randnum);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << interest0.x, interest0.y, interest0.z; // 设置平移        
    transform.rotate(Eigen::AngleAxisf(interest0.yaw, Eigen::Vector3f::UnitY())); // 设置偏航
    transform.rotate(Eigen::AngleAxisf(interest0.pitch, Eigen::Vector3f::UnitX())); // 设置俯仰
    transform.rotate(Eigen::AngleAxisf(interest0.roll, Eigen::Vector3f::UnitZ())); // 设置横滚

    // int i=0; pcl::PointXYZ pointX = {-1,-1,-1};
    for(vector<int>::iterator it = indexP.begin(); it != indexP.end(); ++it)
    {
        pcl::PointXYZ point0 = cloud->points[*it];
        Eigen::Vector3f CandidatedPoint(point0.x, point0.y, point0.z);
        Eigen::Vector3f TransformedPoint = transform.inverse() * CandidatedPoint;
        if(fabs(TransformedPoint.x())<=2 && fabs(TransformedPoint.y())<=1.5 && fabs(TransformedPoint.z())<=1.5)   
            SelectedPointIndices.push_back(*it);
            // interest0.SelectedPoints.push_back(CandidatedPoint);
        // else
        // {
        //     pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud->begin();
        //     index = cloud->begin() + *it;
        //     cloud->erase(index);
        //     cloud->insert(index, pointX);// 这里直接改掉原始点云，下一次循环需要剃掉0点，在新的点云上开始选择点
        // }
    }
    fov_interests.push_back(interest0);
    // interest0.init();
}

//
vector<int> selectViewpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ& search_point) 
{
    //创建KdTree对象
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // 定义搜索半径
    float radius = 1.5; 
    //定义第二次搜索的范围（初始半径的2.5倍）
    float secondSearchRadius = 2.5 * radius;
    int maxNeighborCount = 0;

    //之前的最大点索引
    std::set<int> usedPointIndices;
    //初始化变量以跟踪具有最多邻居点的点
    pcl::PointXYZ maxNeighborPoint;        
    //搜索该点一倍半径内的点
    std::vector<int> firstPointIndices;
    std::vector<float> firstPointDistances;
    //初始化变量以跟踪FovPointIndices firstPointDistances;
    kdtree.radiusSearch(search_point, radius, firstPointIndices, firstPointDistances);
    //搜索该点2.5倍半径内的点
    std::vector<int> secondPointIndices;
    std::vector<float> secondPointDistances;
    kdtree.radiusSearch(search_point, secondSearchRadius, secondPointIndices, secondPointDistances);
    //将覆盖的点放入索引
    usedPointIndices.insert(firstPointIndices.begin(), firstPointIndices.end());
    usedPointIndices.insert(SelectedPointIndices.begin(), SelectedPointIndices.end());

    //剔除之前找到的索引
    for (const int usedIndex : usedPointIndices)
    {
        auto it = std::find(secondPointIndices.begin(), secondPointIndices.end(), usedIndex);
        if (it != secondPointIndices.end())
        {
            secondPointIndices.erase(it);
        }
    }

    //记录每个点的邻居数量
    std::map<int, int> pointNeighborCount;
    //记录最大点的索引
    int maxPointIndex = 0;
    //记录最大点盖住的其他点索引
    std::vector<int> MaxcoverIndices;
    //遍历该点1倍半径外2倍半径内的点 并计算以那个点为半径包含了多少个点
    for (size_t i = 0; i < secondPointIndices.size(); ++i)
    {
        int currentPointIndex = secondPointIndices[i];
        // 如果当前点索引不在第一次搜索结果中，表示在2.5倍半径内但不在1倍半径内
        // 计算第二次搜索半径内的点
        pcl::PointXYZ currentPoint = cloud->points[currentPointIndex];
        // 创建一个用于第二次搜索的新KdTree
        pcl::KdTreeFLANN<pcl::PointXYZ> secondKdTree;
        secondKdTree.setInputCloud(cloud);
        // 在第二次搜索半径内执行半径搜索
        std::vector<int> secondNeighborIndices;
        std::vector<float> secondNeighborDistances;
        secondKdTree.radiusSearch(currentPoint, radius, secondNeighborIndices, secondNeighborDistances);
        // 记录邻居数量
        int neighborCount = secondNeighborIndices.size();
        pointNeighborCount[currentPointIndex] = neighborCount;
        // 找到具有最多邻居的点
        if (neighborCount > maxNeighborCount)
        {
            maxNeighborCount = neighborCount;
            maxNeighborPoint = currentPoint;
            maxPointIndex = currentPointIndex;
            MaxcoverIndices = secondNeighborIndices;
        }
    }
    // std::cout << "search_point after iteration " << j << ": (" << search_point.x << ", " << search_point.y << ", " << search_point.z << ")" << std::endl;     
    //views->addSphere(maxNeighborPoint, 0.2, 0.0, 0.0, 1.0, "MaxNeighborSphere"+ std::to_string(j));
    //std::cout << "currentPointIndex after iteration " << j << ": " << maxPointIndex << std::endl;
    FovPointIndices.insert(maxPointIndex);   

    MaxcoverIndices.push_back(maxPointIndex);
    search_point = maxNeighborPoint;
    if(maxPointIndex==0)//待优化
        for (const int usedIndex : usedPointIndices)
        {
            auto it = std::find(AllpointIndices.begin(), AllpointIndices.end(), usedIndex);
            if (it != AllpointIndices.end())
            {
                AllpointIndices.erase(it);
            }
        }
        search_point=cloud->points[AllpointIndices[0]]; 
    return MaxcoverIndices;
}

// void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) 
// {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromROSMsg(*cloud_msg, *cloud);

//     auto start_time = std::chrono::high_resolution_clock::now();//开始计运行时间
    
//     //设置降采样滤波器
//     downsamplePointCloud(cloud);

//     for (int i = 0; i < cloud->size(); ++i) {
//         AllpointIndices.push_back(i);
//     }

//     //计算测试物体的中心坐标
//     Eigen::Vector4f centroid;
//     pcl::compute3DCentroid(*cloud, centroid);
//     pcl::PointXYZ Model_center(centroid[0], centroid[1], centroid[2]);//转换中心坐标格式

//     //计算法向量
//     pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
//     computeNormals(cloud, cloud_normals);
//     //校正法向量方向
//     correctNormals(centroid, cloud, cloud_normals);
//     std::cout << "法向量最大索引:" << cloud_normals->size() << std::endl;

//     int num = rand() % (cloud->size()); pcl::PointXYZ search_point = cloud->points[num];//待优化
//     std::vector<int> test;
//     for(int j=0;j<cloud->size();j++)// 可优化循环次数
//     {
//         test = selectViewpoint(cloud, search_point); 
//         selectPointoffov(cloud, cloud_normals, test);
//     }

//     auto end_time = std::chrono::high_resolution_clock::now();//结束计时
//     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
//     std::cout << "程序运行时间：" << duration.count() << " 毫秒" << std::endl;

//     //可视化
//     pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//     viewer->setBackgroundColor (0, 0, 0);
//     viewer->addPointCloud<pcl::PointXYZ> (cloud,"sample cloud");//显示点云
//     viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud");//设置点的大小
//     viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, cloud_normals, 1, 1, "normals");//显示法向量
//     viewer->addSphere(Model_center, 0.2, 1.0, 0.0, 0.0, "center_sphere");//显示物体中心
//     // test_Fov(viewer, cloud, cloud_normals, fov_interests);
//     show_fov(viewer, fov_interests);//根据视点打印所有fov
//     viewer->addCoordinateSystem (1.0);
//     viewer->initCameraParameters ();
//     while (!viewer->wasStopped ())
//     {
//         viewer->spinOnce (100);
//         std::this_thread::sleep_for(100ms);
//     };
//     rate.sleep(); 
// }

class pcl_sub
{
private:
  ros::NodeHandle n;
  ros::Subscriber subCloud;
  ros::Publisher pubCloud;
  ros::Rate rate;
  sensor_msgs::PointCloud2 msg;  //接收到的点云消息
  sensor_msgs::PointCloud2 adjust_msg;  //等待发送的点云消息
  pcl::PointCloud<pcl::PointXYZ> adjust_pcl;   //建立了一个pcl的点云，作为中间过程

public:
  pcl_sub():
    n("~"), rate(1){
    subCloud = n.subscribe<sensor_msgs::PointCloud2>("/pcl_output", 1, &pcl_sub::getcloud, this); //接收点云数据，进入回调函数getcloud()
    // pubCloud = n.advertise<sensor_msgs::PointCloud2>("/adjustd_cloud", 1000);  //建立了一个发布器，主题是/adjusted_cloud，方便之后发布调整后的点云
  }

  //回调函数
  void getcloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    auto start_time = std::chrono::high_resolution_clock::now();//开始计运行时间
    
    //设置降采样滤波器
    downsamplePointCloud(cloud);

    for (int i = 0; i < cloud->size(); ++i) {
        AllpointIndices.push_back(i);
    }

    //计算测试物体的中心坐标
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    pcl::PointXYZ Model_center(centroid[0], centroid[1], centroid[2]);//转换中心坐标格式

    //计算法向量
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    computeNormals(cloud, cloud_normals);
    //校正法向量方向
    correctNormals(centroid, cloud, cloud_normals);
    std::cout << "法向量最大索引:" << cloud_normals->size() << std::endl;

    int num = rand() % (cloud->size()); pcl::PointXYZ search_point = cloud->points[num];//待优化
    std::vector<int> indexcover;
    for(int j=0;j<cloud->size();j++)// 可优化循环次数
    {
        indexcover = selectViewpoint(cloud, search_point); 
        selectPointoffov(cloud, cloud_normals, indexcover);
    }

    auto end_time = std::chrono::high_resolution_clock::now();//结束计时
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "程序运行时间：" << duration.count() << " 毫秒" << std::endl;

    //可视化
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");//显示点云
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud");//设置点的大小
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, cloud_normals, 1, 1, "normals");//显示法向量
    viewer->addSphere(Model_center, 0.2, 1.0, 0.0, 0.0, "center_sphere");//显示物体中心
    // test_Fov(viewer, cloud, cloud_normals, fov_interests);
    show_fov(viewer, fov_interests);//根据视点打印所有fov
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        // std::this_thread::sleep_for(100ms);
    };
    rate.sleep(); 
  }

  ~pcl_sub(){}
};

int main(int argc, char** argv) 
{
    // ros::init(argc, argv, "point_cloud_processing_node");
    // ros::NodeHandle nh;
    // ros::Rate rate(0.1);
    // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, pointCloudCallback);
    // ros::spin();
    // return 0;

    ros::init(argc, argv, "colored");  //初始化了一个节点，名字为colored
    pcl_sub ps;
    ros::spin();
    return 0;
}
//在移植进入比赛的模型中时 要给定bonund box的中心点 对法向量进行重定向 否则FOV会算错方向 在测试时使用质心代替