#include <ros/ros.h>
#include <thread>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <visualization_msgs/Marker.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <string>
#include <chrono>
#include <cstdlib>
using namespace std::chrono_literals;
float Fov_Really_size =0;   //视点的最大索引数值
float fov_interst[9000][6];//装视点的容器
double fov_horizontal_deg = 60.0; // 水平FOV（度）
double fov_vertical_deg = 45.0;   // 垂直FOV（度）
double fov_distance =1;          //FOV的长度
std::set<int> FOVPointIndices;  //FOV索引
void Show_fov(pcl::visualization::PCLVisualizer::Ptr views)
{
    for(int i=0;i<Fov_Really_size;i=i+40)
    {
        //输入数据
        float x=fov_interst[i][0];
        float y=fov_interst[i][1];
        float z=fov_interst[i][2];
        float pitch_deg=fov_interst[i][3];
        float yaw_deg=fov_interst[i][4];
        float roll_deg=fov_interst[i][5];
        // 将角度转换为弧度
        double pitch_rad = pitch_deg * M_PI / 180.0;
        double yaw_rad = yaw_deg * M_PI / 180.0;
        double roll_rad = roll_deg * M_PI / 180.0;
        // 创建一个变换矩阵来表示位姿
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << x, y, z; // 设置平移
        transform.rotate(Eigen::AngleAxisf(yaw_rad, Eigen::Vector3f::UnitY())); // 设置偏航
        transform.rotate(Eigen::AngleAxisf(pitch_rad, Eigen::Vector3f::UnitX())); // 设置俯仰
        transform.rotate(Eigen::AngleAxisf(roll_rad, Eigen::Vector3f::UnitZ())); // 设置横滚
        // 计算FOV的边界点
        double fov_half_width = std::tan(fov_horizontal_deg / 2.0 * M_PI / 180.0) * fov_distance;
        double fov_half_height = std::tan(fov_vertical_deg / 2.0 * M_PI / 180.0) * fov_distance;

        Eigen::Vector3f fov_center(0.0, 0.0, fov_distance); // FOV中心点坐标
        Eigen::Vector3f fov_top_left(-fov_half_width, fov_half_height, fov_distance);
        Eigen::Vector3f fov_top_right(fov_half_width, fov_half_height, fov_distance);
        Eigen::Vector3f fov_bottom_left(-fov_half_width, -fov_half_height, fov_distance);
        Eigen::Vector3f fov_bottom_right(fov_half_width, -fov_half_height, fov_distance);
        Eigen::Vector3f fov_base(x,y,z);

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
        views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_top_left.x(), fov_top_left.y(), fov_top_left.z()), pcl::PointXYZ(fov_top_right.x(), fov_top_right.y(), fov_top_right.z()), 1.0, 0.0, 0.0,temp_str1);
        views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_top_right.x(), fov_top_right.y(), fov_top_right.z()), pcl::PointXYZ(fov_bottom_right.x(), fov_bottom_right.y(), fov_bottom_right.z()), 1.0, 0.0, 0.0,temp_str2);
        views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_bottom_right.x(), fov_bottom_right.y(), fov_bottom_right.z()), pcl::PointXYZ(fov_bottom_left.x(), fov_bottom_left.y(), fov_bottom_left.z()), 1.0, 0.0, 0.0,temp_str3);
        views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_bottom_left.x(), fov_bottom_left.y(), fov_bottom_left.z()), pcl::PointXYZ(fov_top_left.x(), fov_top_left.y(), fov_top_left.z()), 1.0, 0.0, 0.0, temp_str4);

        views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_base.x(), fov_base.y(), fov_base.z()), pcl::PointXYZ(fov_top_left.x(), fov_top_left.y(), fov_top_left.z()), 1.0, 0.0, 0.0,temp_str5);
        views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_base.x(), fov_base.y(), fov_base.z()), pcl::PointXYZ(fov_top_right.x(), fov_top_right.y(), fov_top_right.z()), 1.0, 0.0, 0.0,temp_str6);
        views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_base.x(), fov_base.y(), fov_base.z()), pcl::PointXYZ(fov_bottom_left.x(), fov_bottom_left.y(), fov_bottom_left.z()), 1.0, 0.0, 0.0,temp_str7);
        views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_base.x(), fov_base.y(), fov_base.z()), pcl::PointXYZ(fov_bottom_right.x(), fov_bottom_right.y(), fov_bottom_right.z()), 1.0, 0.0, 0.0,temp_str8);
    }
}
//
void Show_fov_Index(pcl::visualization::PCLVisualizer::Ptr views,int mode)
{
    int i=0;
    //for(int i=0;i<Fov_Really_size;i=i+40)
    for(int index : FOVPointIndices)
    {
        i++;
        //输入数据
        float x=fov_interst[index][0];
        float y=fov_interst[index][1];
        float z=fov_interst[index][2];
        float pitch_deg=fov_interst[index][3];
        float yaw_deg=fov_interst[index][4];
        float roll_deg=fov_interst[index][5];
        // 将角度转换为弧度
        double pitch_rad = pitch_deg * M_PI / 180.0;
        double yaw_rad = yaw_deg * M_PI / 180.0;
        double roll_rad = roll_deg * M_PI / 180.0;
        // 创建一个变换矩阵来表示位姿
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << x, y, z; // 设置平移
        transform.rotate(Eigen::AngleAxisf(yaw_rad, Eigen::Vector3f::UnitY())); // 设置偏航
        transform.rotate(Eigen::AngleAxisf(pitch_rad, Eigen::Vector3f::UnitX())); // 设置俯仰
        transform.rotate(Eigen::AngleAxisf(roll_rad, Eigen::Vector3f::UnitZ())); // 设置横滚
        // 计算FOV的边界点
        double fov_half_width = std::tan(fov_horizontal_deg / 2.0 * M_PI / 180.0) * fov_distance;
        double fov_half_height = std::tan(fov_vertical_deg / 2.0 * M_PI / 180.0) * fov_distance;

        Eigen::Vector3f fov_center(0.0, 0.0, fov_distance); // FOV中心点坐标
        Eigen::Vector3f fov_top_left(-fov_half_width, fov_half_height, fov_distance);
        Eigen::Vector3f fov_top_right(fov_half_width, fov_half_height, fov_distance);
        Eigen::Vector3f fov_bottom_left(-fov_half_width, -fov_half_height, fov_distance);
        Eigen::Vector3f fov_bottom_right(fov_half_width, -fov_half_height, fov_distance);
        Eigen::Vector3f fov_base(x,y,z);

        // 应用位姿变换到FOV边界点
        fov_center = transform * fov_center;
        fov_top_left = transform * fov_top_left;
        fov_top_right = transform * fov_top_right;
        fov_bottom_left = transform * fov_bottom_left;
        fov_bottom_right = transform * fov_bottom_right;
        if(mode)
            views->addSphere(pcl::PointXYZ(x, y, z), 0.2, 0.0, 0.0, 1.0, "sep"+ std::to_string(i));
        else
        {
            // 绘制FOV的边界线
            std::string temp_str1="fova";temp_str1+=std::to_string(i);std::string temp_str2="fovb";temp_str2+=std::to_string(i);
            std::string temp_str3="fovc";temp_str3+=std::to_string(i);std::string temp_str4="fovd";temp_str4+=std::to_string(i);
            std::string temp_str5="fove";temp_str5+=std::to_string(i);std::string temp_str6="fovf";temp_str6+=std::to_string(i);
            std::string temp_str7="fovg";temp_str7+=std::to_string(i);std::string temp_str8="fovh";temp_str8+=std::to_string(i);
            views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_top_left.x(), fov_top_left.y(), fov_top_left.z()), pcl::PointXYZ(fov_top_right.x(), fov_top_right.y(), fov_top_right.z()), 1.0, 0.0, 0.0,temp_str1);
            views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_top_right.x(), fov_top_right.y(), fov_top_right.z()), pcl::PointXYZ(fov_bottom_right.x(), fov_bottom_right.y(), fov_bottom_right.z()), 1.0, 0.0, 0.0,temp_str2);
            views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_bottom_right.x(), fov_bottom_right.y(), fov_bottom_right.z()), pcl::PointXYZ(fov_bottom_left.x(), fov_bottom_left.y(), fov_bottom_left.z()), 1.0, 0.0, 0.0,temp_str3);
            views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_bottom_left.x(), fov_bottom_left.y(), fov_bottom_left.z()), pcl::PointXYZ(fov_top_left.x(), fov_top_left.y(), fov_top_left.z()), 1.0, 0.0, 0.0, temp_str4);

            views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_base.x(), fov_base.y(), fov_base.z()), pcl::PointXYZ(fov_top_left.x(), fov_top_left.y(), fov_top_left.z()), 1.0, 0.0, 0.0,temp_str5);
            views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_base.x(), fov_base.y(), fov_base.z()), pcl::PointXYZ(fov_top_right.x(), fov_top_right.y(), fov_top_right.z()), 1.0, 0.0, 0.0,temp_str6);
            views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_base.x(), fov_base.y(), fov_base.z()), pcl::PointXYZ(fov_bottom_left.x(), fov_bottom_left.y(), fov_bottom_left.z()), 1.0, 0.0, 0.0,temp_str7);
            views->addLine<pcl::PointXYZ>(pcl::PointXYZ(fov_base.x(), fov_base.y(), fov_base.z()), pcl::PointXYZ(fov_bottom_right.x(), fov_bottom_right.y(), fov_bottom_right.z()), 1.0, 0.0, 0.0,temp_str8);
        }
        
    }
}
void Test_Fov(pcl::visualization::PCLVisualizer::Ptr views,pcl::PointCloud<pcl::PointXYZ>::Ptr clouds,pcl::PointCloud<pcl::Normal>::Ptr normals)
{
        int num =364;
        //输入数据
        float x=fov_interst[num][0];
        float y=fov_interst[num][1];
        float z=fov_interst[num][2];
        float pitch_deg=fov_interst[num][3];
        float yaw_deg=fov_interst[num][4];
        float roll_deg=fov_interst[num][5];

        std::cout << "测试的 俯仰 偏航: (" << pitch_deg << ", " << yaw_deg<< ")" << std::endl;
        pcl::Normal normal = normals->points[num];
        double nx = normal.normal_x;
        double ny = normal.normal_y;
        double nz = normal.normal_z;
        // 输出法向量信息
        std::cout << "Normal: (" << nx << ", " << ny << ", " << nz << ")" << std::endl;
        // 将角度转换为弧度
        double pitch_rad = pitch_deg * M_PI / 180.0;
        double yaw_rad = yaw_deg * M_PI / 180.0;
        double roll_rad = roll_deg * M_PI / 180.0;
        // 创建一个变换矩阵来表示位姿
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << x, y, z; // 设置平移        
        transform.rotate(Eigen::AngleAxisf(yaw_rad, Eigen::Vector3f::UnitY())); // 设置偏航
        transform.rotate(Eigen::AngleAxisf(pitch_rad, Eigen::Vector3f::UnitX())); // 设置俯仰
        transform.rotate(Eigen::AngleAxisf(roll_rad, Eigen::Vector3f::UnitZ())); // 设置横滚
        // 计算FOV的边界点
        double fov_half_width = std::tan(fov_horizontal_deg / 2.0 * M_PI / 180.0) * fov_distance;
        double fov_half_height = std::tan(fov_vertical_deg / 2.0 * M_PI / 180.0) * fov_distance;

        Eigen::Vector3f fov_center(0.0, 0.0, fov_distance); // FOV中心点坐标
        Eigen::Vector3f fov_top_left(-fov_half_width, fov_half_height, fov_distance);
        Eigen::Vector3f fov_top_right(fov_half_width, fov_half_height, fov_distance);
        Eigen::Vector3f fov_bottom_left(-fov_half_width, -fov_half_height, fov_distance);
        Eigen::Vector3f fov_bottom_right(fov_half_width, -fov_half_height, fov_distance);
        Eigen::Vector3f fov_base(x,y,z);

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
}
void compute_fov(const pcl::Normal& normal,const pcl::PointXYZ& start_point,float fov_array[6])
{
    // 计算法向量的方向向量
    Eigen::Vector3d view_direction(normal.normal_x, normal.normal_y, normal.normal_z);
    view_direction.normalize();
    // 计算FOV的位置（xyz坐标）
    Eigen::Vector3d fov_position(start_point.x - fov_distance * view_direction.x(),
                                start_point.y - fov_distance * view_direction.y(),
                                start_point.z - fov_distance * view_direction.z());
    fov_array[0]=fov_position.x();
    fov_array[1]=fov_position.y();
    fov_array[2]=fov_position.z();
    // 计算俯仰角 
    //fov_array[3] = std::asin((view_direction.y())); // 使用反正弦函数计算
    fov_array[3] = std::atan2(view_direction.y(),sqrt(view_direction.x()*view_direction.x()+view_direction.z()*view_direction.z())); 
    fov_array[3] = -fov_array[3] * 180.0 / M_PI; // 俯仰角转换为度
    // 计算偏航角
    fov_array[4] = std::atan2(view_direction.x(),view_direction.z());
    fov_array[4] = fov_array[4] * 180.0 / M_PI; // 偏航角转换为度
    // 设置横滚角为0 现在测试打开;
    fov_array[5] = 0;
} 
void Get_all_Fov(pcl::PointCloud<pcl::PointXYZ>::Ptr clouds,pcl::PointCloud<pcl::Normal>::Ptr cloud_normal)//输入点云和法向量 输出兴趣点数组
{
    // 输出法向量和坐标信息
    for (std::size_t i = 0; i < cloud_normal->size(); ++i) 
    {
        pcl::Normal normal = cloud_normal->points[i];
        //访问当前点云信息
        pcl::PointXYZ point = clouds->points[i];
        // 访问法向量的坐标分量
        double nx = normal.normal_x;
        double ny = normal.normal_y;
        double nz = normal.normal_z;
        //计算当前点和法向量对应的视点
        compute_fov(normal,point,fov_interst[i]);
    }
    Fov_Really_size=cloud_normal->size();
}

void selectViewpoint(pcl::visualization::PCLVisualizer::Ptr views,const pcl::PointCloud<pcl::PointXYZ>::Ptr clouds,pcl::PointCloud<pcl::Normal>::Ptr normals,pcl::PointCloud<pcl::PointXYZ>::Ptr& viewpoints,pcl::PointCloud<pcl::Normal>::Ptr& viewpointNormals) 
{
    //创建KdTree对象
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(clouds);
    //创建一个临时点云用于修改
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = *clouds;

    int num = rand() % (cloud->size());
    // 索引随机点的坐标
    pcl::PointXYZ search_point = cloud->points[num];
    //之前的最大点索引
    std::set<int> usedPointIndices;
    // 定义搜索半径
    float radius = 1.5; 
    //定义第二次搜索的范围（初始半径的2.5倍）
    float secondSearchRadius = 2.5 * radius;
    for(int j=0;j<300;j++)
    {
        //初始化变量以跟踪具有最多邻居点的点
        int maxNeighborCount = 0;
        pcl::PointXYZ maxNeighborPoint;        
        //搜索该点一倍半径内的点
        std::vector<int> firstPointIndices;
        std::vector<float> firstPointDistances;
        kdtree.radiusSearch(search_point, radius, firstPointIndices, firstPointDistances);
        //将覆盖的点放入索引
        usedPointIndices.insert(firstPointIndices.begin(), firstPointIndices.end());
        //搜索该点2.5倍半径内的点
        std::vector<int> secondPointIndices;
        std::vector<float> secondPointDistances;
        kdtree.radiusSearch(search_point, secondSearchRadius, secondPointIndices, secondPointDistances);
        //剔除之前找到的最大索引
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
            if (std::find(firstPointIndices.begin(), firstPointIndices.end(), currentPointIndex) == firstPointIndices.end())
            {
                // 计算第二次搜索半径内的点
                pcl::PointXYZ currentPoint = cloud->points[currentPointIndex];
                // 创建一个用于第二次搜索的新KdTree
                pcl::KdTreeFLANN<pcl::PointXYZ> secondKdTree;
                secondKdTree.setInputCloud(cloud);
                // 在第二次搜索半径内执行半径搜索
                std::vector<int> secondNeighborIndices;
                std::vector<float> secondNeighborDistances;
                secondKdTree.radiusSearch(currentPoint, radius, secondNeighborIndices, secondNeighborDistances);
                //排出掉之前已经覆盖过的
                for (size_t k = 0; k < firstPointIndices.size(); ++k)
                {
                   int excludedPointIndex = firstPointIndices[k];
                   secondNeighborIndices.erase(std::remove(secondNeighborIndices.begin(), secondNeighborIndices.end(), excludedPointIndex), secondNeighborIndices.end());
                }
                for (const int usedIndex : usedPointIndices)
                {
                    auto it = std::find(secondNeighborIndices.begin(), secondNeighborIndices.end(), usedIndex);
                    if (it != secondNeighborIndices.end())
                    {
                        secondNeighborIndices.erase(it);
                    }
                }
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
        }
        std::cout << "search_point after iteration " << j << ": (" << search_point.x << ", " << search_point.y << ", " << search_point.z << ")" << std::endl;
        search_point = maxNeighborPoint; 
        //记录所有fov索引        
        if(maxPointIndex==0)
            break;
        FOVPointIndices.insert(maxPointIndex);   
        //views->addSphere(maxNeighborPoint, 0.2, 0.0, 0.0, 1.0, "MaxNeighborSphere"+ std::to_string(j));
        std::cout << "currentPointIndex after iteration " << j << ": " << maxPointIndex << std::endl;
    }
}

//校正法向量
void correctNormals(Eigen::Vector4f centroids,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals) 
{
    // 校正法向量方向
    for (size_t i = 0; i < normals->size(); ++i) 
    {
        Eigen::Vector3f vector_to_centroid(centroids[0] - cloud->points[i].x,
                                           centroids[1] - cloud->points[i].y,
                                           centroids[2] - cloud->points[i].z);
        // 点积为负值表示法向量指向外部，翻转法向量
        if (normals->points[i].getNormalVector3fMap().dot(vector_to_centroid) < 0) {
            normals->points[i].getNormalVector3fMap() *= -1;
        }
    }
}

//降采样
void downsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.7, 0.7, 0.7); // 设置降采样的体素大小
    sor.filter(*cloud);
}

//计算法向量
void computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, double radius = 8.0)
{
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(radius); // 设置搜索半径，根据你的数据调整
    ne.compute(*cloud_normals);
}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) 
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    //计算测试物体的中心坐标
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    pcl::PointXYZ Model_center(centroid[0], centroid[1], centroid[2]);//转换中心坐标格式
    
    //设置降采样滤波器
    downsamplePointCloud(cloud);
    //计算法向量
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    computeNormals(cloud, cloud_normals);
    //校正法向量方向
    correctNormals(centroid,cloud,cloud_normals);
    std::cout << "法向量最大索引:" << cloud_normals->size() << std::endl;
    //根据策略筛选
    pcl::PointCloud<pcl::PointXYZ>::Ptr viewpoints; 
    pcl::PointCloud<pcl::Normal>::Ptr viewpointNormals;
    //std::cout << "筛选后视点最大索引:" << viewpoints->size() << std::endl;
    //根据法向量点云计算出视点
    Get_all_Fov(cloud,cloud_normals);

    //可视化
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud,"sample cloud");//显示点云
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");//设置点的大小
    //viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, cloud_normals, 1, 0.25, "normals");//显示法向量
    viewer->addSphere(Model_center, 1.5, 1.0, 0.0, 0.0, "center_sphere");//显示物体中心
    //Test_Fov(viewer,cloud,cloud_normals);
    auto start_time = std::chrono::high_resolution_clock::now();//开始计运行时间
    selectViewpoint(viewer,cloud,cloud_normals,viewpoints,viewpointNormals);    
    //打印计算时间 输出运行时间（以毫秒为单位）
    auto end_time = std::chrono::high_resolution_clock::now();//结束计时
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "程序运行时间：" << duration.count() << " 毫秒" << std::endl;
    Show_fov_Index(viewer,1);
    //Show_fov(viewer);//根据视点打印所有fov
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }
    while(1);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "point_cloud_processing_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/pcl_output", 1, pointCloudCallback);

    ros::spin();
    return 0;
}
//在移植进入比赛的模型中时 要给定bonund box的中心点 对法向量进行重定向 否则FOV会算错方向 在测试时使用质心代替