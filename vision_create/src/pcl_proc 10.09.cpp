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
using std::vector;
using namespace std::chrono_literals;
float Fov_Really_size = 0;  // 视点的最大索引数值
double fov_horizontal_deg = 60.0;  // 水平FOV（度）
double fov_vertical_deg = 45.0;  // 垂直FOV（度）
double fov_distance =1.0;
double fov_half_width = std::tan(fov_horizontal_deg / 2.0 * M_PI / 180.0) * fov_distance;  // 计算FOV的边界点
double fov_half_height = std::tan(fov_vertical_deg / 2.0 * M_PI / 180.0) * fov_distance;

class FoVInterest {
public:
    float x;
    float y;
    float z;
    float pitch;
    float yaw;
    float roll;
    vector<Eigen::Vector3f> SelectedPoints;

void init() {
    x = 0.0;
    y = 0.0;
    z = 0.0;
    pitch = 0.0;
    yaw = 0.0;
    roll = 0.0;
    }
};// 装视点的容器
FoVInterest interest0;
vector<FoVInterest> fov_interests;

void show_fov(pcl::visualization::PCLVisualizer::Ptr views, vector<FoVInterest>& vec)
{
    int i=0; int skipCount = 0;
    for(vector<FoVInterest>::iterator it = vec.begin(); it != vec.end(); ++it, ++i)
    {
        if (skipCount < 15) 
        {
            skipCount++;
            continue;
        }
        else skipCount = 0;
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
    fov_interests.push_back(interest);
    interest.init();
} 

void get_all_fov(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normal, FoVInterest& interest)
{
    // 输出法向量和坐标信息
    for (std::size_t i = 0; i < cloud_normal->size(); ++i) 
    {
        pcl::Normal normal = cloud_normal->points[i];
        //访问当前点云信息
        pcl::PointXYZ point = cloud->points[i];
        // 访问法向量的坐标分量
        double nx = normal.normal_x;
        double ny = normal.normal_y;
        double nz = normal.normal_z;
        //计算当前点和法向量对应的视点
        compute_fov(normal, point, interest);
    }
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
void computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr& incloud, pcl::PointCloud<pcl::Normal>::Ptr& cloud_normal, double radius = 8.0)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(incloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(radius); // 设置搜索半径，根据你的数据调整
    ne.compute(*cloud_normal);
}

//判断是否超出FoV
void selectPointoffov(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, vector<int>& indexP) 
{   
    int randnum = indexP[0];
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << fov_interests[randnum].x, fov_interests[randnum].y, fov_interests[randnum].z; // 设置平移        
    transform.rotate(Eigen::AngleAxisf(fov_interests[randnum].yaw, Eigen::Vector3f::UnitY())); // 设置偏航
    transform.rotate(Eigen::AngleAxisf(fov_interests[randnum].pitch, Eigen::Vector3f::UnitX())); // 设置俯仰
    transform.rotate(Eigen::AngleAxisf(fov_interests[randnum].roll, Eigen::Vector3f::UnitZ())); // 设置横滚

    int i=0; pcl::PointXYZ pointX = {-1,-1,-1};
    for(vector<int>::iterator it = indexP.begin(); it != indexP.end(); ++it, ++i)
    {
        pcl::PointXYZ point0 = cloud->points[*it];
        Eigen::Vector3f CandidatedPoint(point0.x, point0.y, point0.z);
        Eigen::Vector3f TransformedPoint = transform.inverse() * CandidatedPoint;
        if(fabs(TransformedPoint.x())<=2 && fabs(TransformedPoint.y())<=1.5 && fabs(TransformedPoint.z())<=1.5)   
            interest0.SelectedPoints.push_back(TransformedPoint);
        else
        {
            pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud->begin();
            index = cloud->begin() + *it;
            cloud->erase(index);
            cloud->insert(index, pointX);// 这里直接改掉原始点云，下一次循环需要剃掉0点，在新的点云上开始选择点
        }
    }
}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) 
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    auto start_time = std::chrono::high_resolution_clock::now();//开始计运行时间
    
    //设置降采样滤波器
    downsamplePointCloud(cloud);

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

    //根据法向量点云计算出视点
    get_all_fov(cloud, cloud_normals, interest0);

    //
    vector<int> test={303,165,1,304,305,306,307,308};
    selectPointoffov(cloud, test);

    auto end_time = std::chrono::high_resolution_clock::now();//结束计时
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "程序运行时间：" << duration.count() << " 毫秒" << std::endl;

    //可视化
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud,"sample cloud");//显示点云
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud");//设置点的大小
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, cloud_normals, 1, 1, "normals");//显示法向量
    viewer->addSphere(Model_center, 0.2, 1.0, 0.0, 0.0, "center_sphere");//显示物体中心
    test_Fov(viewer, cloud, cloud_normals, fov_interests);
    // show_fov(viewer, fov_interests);//根据视点打印所有fov
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