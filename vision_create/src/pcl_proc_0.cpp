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
#include <yaml-cpp/yaml.h>
#include <string>
#include <chrono>
#include <random>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Dense>
#include <limits>
#include <list>
#include <algorithm> 
#include <deque>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
using namespace std::chrono_literals;
using namespace std;
class TSP_Slover
{
    public:
    TSP_Slover(){} 
    TSP_Slover(Eigen::Vector3d agentpose, list<Eigen::Vector3d> viewpoint , int slover_method)  //从无人机的位置到达所有的视点
    {
        //将无人机的位置加入视点列表
        list<Eigen::Vector3d> viewpoint_ = viewpoint;
        viewpoint_.push_front(agentpose);
        //计算带起始点约束的TSP问题
        switch (slover_method)
        {
        case 1:
            cout << "dynamic plan algorithm" << endl;
            waypoint = calculate_sort_viewpoint_dynamic(viewpoint_,false); break;
        case 2:
            cout << "greedy algorithm" << endl;
            waypoint = calculate_sort_viewpoint_greedy(viewpoint_,false); break;
        default:
            break;
        }
    }

    TSP_Slover(Eigen::Vector3d agentpose, list<Eigen::Vector3d> viewpoint, Eigen::Vector3d lastpose) //从无人机的位置到达所有视点并指定终点位置
    {
        //将无人机的位置和终点位置加入视点列表
        list<Eigen::Vector3d> viewpoint_ = viewpoint;
        viewpoint_.push_front(agentpose);
        viewpoint_.push_back(lastpose);
        //计算带起始点约束和终点约束的TSP问题
        waypoint = calculate_sort_viewpoint_dynamic(viewpoint_,true);
    }

    list<Eigen::Vector3d> get_waypoint()
    {
        list<Eigen::Vector3d> res;
        res = waypoint;
        return res;
    }
    /*可视化视点和路径连接*/
    visualization_msgs::MarkerArray visualization_msg(list<Eigen::Vector3d>& _waypoint)
    {
        geometry_msgs::Point pose;
        visualization_msgs::MarkerArray result;
        visualization_msgs::Marker _points,line;
        _points.header.frame_id = "odom";
        _points.header.stamp = ros::Time();
        _points.ns = "my_namespace";
        _points.id = 1;
        _points.action = visualization_msgs::Marker::ADD;
        _points.pose.orientation.w = 1.0;
        _points.scale.x = 1;
        _points.scale.y = 1;
        _points.scale.z = 1;
        _points.color.a = 1.0; // Don't forget to set the alpha!
        _points.color.r = 0.0;
        _points.color.g = 1.0;
        _points.color.b = 0.0;
        _points.type = visualization_msgs::Marker::POINTS;
        line.header.frame_id = "odom";
        line.header.stamp = _points.header.stamp;
        line.ns = "my_namespace";
        line.id = 2;
        line.action = visualization_msgs::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x = 0.5;
        line.scale.y = 0.5;
        line.scale.z = 0.5;
        line.color.a = 1.0; // Don't forget to set the alpha!
        line.color.r = 1.0;
        line.color.g = 0.0;
        line.color.b = 0.0;
        line.type = visualization_msgs::Marker::LINE_STRIP;  //显示路径段
        for ( const Eigen::Vector3d&res : _waypoint)
        {
            pose.x = res[0];
            pose.y = res[1];
            pose.z = res[2];
            _points.points.push_back(pose);
            line.points.push_back(pose);
        }
        result.markers.push_back(_points);
        result.markers.push_back(line);
        return result;
    }
     
    private:
    visualization_msgs::MarkerArray visual_able;
    list<Eigen::Vector3d> waypoint; //视点排列完成后的路径点序列
    std::vector<std::vector<int>> viewpoint_distant_Mat ; //视点列表的距离矩阵
    std::vector<std::vector<double>> calculate_distant_mat(list<Eigen::Vector3d>& reviewpoint)  //输入视点，计算距离矩阵
    {        
        std::vector<Eigen::Vector3d> viewpoint_;
        std::vector<std::vector<double>> result;
        for (const Eigen::Vector3d& element : reviewpoint) 
        {
            viewpoint_.push_back(element);
        }
        const int viewpoint_num = viewpoint_.size();
        double arry[viewpoint_num][viewpoint_num];
        for (int i=0 ; i<viewpoint_num ; i++)
        {
            for (int j=i ; j<viewpoint_num ; j++)
            {
                if( i == j )  { arry[i][j] = INT_MAX; }
                else {arry[i][j] = (viewpoint_[i]-viewpoint_[j]).norm();   arry[j][i] = arry[i][j] ; }
            }
        }
        for (int i = 0; i < viewpoint_num; i++) 
        {
            std::vector<double> row;
            for (int j = 0; j < viewpoint_num; j++)  
                row.push_back(arry[i][j]);
            result.push_back(row);
        }
        return result;
    }

   void generateSubsets(const std::vector<int>& originalSet, std::vector<int>& currentSubset, int index,std::vector<std::vector<int>>& Subset) 
   {
        if (index == originalSet.size()) 
        {
            Subset.push_back(currentSubset);
            return;
        }
        // 不包含当前元素的子集
        generateSubsets(originalSet, currentSubset, index + 1,Subset);
        // 包含当前元素的子集
        currentSubset.push_back(originalSet[index]);
        generateSubsets(originalSet, currentSubset, index + 1,Subset);
        // 恢复当前子集以继续生成其他子集
        currentSubset.pop_back();
    }
    
    /*自定义排序比较函数*/
    static bool customSort(const std::vector<int>& a, const std::vector<int>& b) 
    {
        if (a.size() < b.size()) 
        {
            return true;
        }
        if (a.size() > b.size()) 
        {
            return false;
        }
        return a[0] > b[0];
    }

    list<Eigen::Vector3d> calculate_sort_viewpoint_dynamic(list<Eigen::Vector3d> reviewpoint, bool is_fixed_lastpoint)  //动态规划排列后的视点顺序路径点
    {
        list<Eigen::Vector3d> result;
        int waypoint_num;
        std::vector<std::vector<double>> distant_mat;
        distant_mat = calculate_distant_mat (reviewpoint);     //计算距离矩阵
        /*打印距离矩阵*/
        // for (int i=0 ; i<distant_mat.size();i++)
        // {
        //     for (int j=0 ; j<distant_mat[i].size() ; j++)
        //         cout << distant_mat[i][j] << "  ";
        //     cout << endl;
        // }
        /*动态规划*/
        // std::deque<int> waypoint_flag; //路径点顺序标记
        std::vector<std::vector<double>>  cost ; //代价矩阵
        std::vector<std::vector<int>> Subset;  //路径点集合所有子集
        std::vector<int> originalSet ;
        std::vector<int> currentSubset;   
        waypoint_num = distant_mat.size();   //路径点个数
        for (int i=1 ; i < waypoint_num ; i++)  originalSet.push_back(i);  //初始化子集序列
        generateSubsets(originalSet, currentSubset, 0,Subset);//路径点集合所有子集Subset
        sort(Subset.begin(), Subset.end(), customSort);
        /*打印子集*/
        // for (int i=0 ; i<Subset.size();i++)
        // {
        //     for (int j=0 ; j<Subset[i].size() ; j++)
        //         cout << Subset[i][j] << "  ";
        //     cout << "  ;  ";
        // }
        // cout <<endl;
        std::vector<std::vector<double>> dp;
        double dp_re[waypoint_num][(int)pow(2,waypoint_num-1)];
        int pre[waypoint_num][(int)pow(2,waypoint_num-1)];
        // for(int i=1;i<waypoint_num;i++) dp[i][0] = distant_mat[i][0]; //初始化第0列
        for(int i=1;i<waypoint_num;i++) dp_re[i][0] = 0;   //不需要回到出发点，代价记为0
        std::vector<int> set;
        int index_;
        for (int j=1 ; j<pow(2,waypoint_num-1)-1 ; j++)   /*计算dp*/
        {
            for(int i=1; i<waypoint_num; i++) 
            {                
                if(std::find(Subset[j].begin(), Subset[j].end(), i) == Subset[j].end()) //子集中不包含元素
                {
                    dp_re[i][j] = INT16_MAX;
                    pre[i][j] = Subset[j][0];
                    for (int k=0 ; k<Subset[j].size() ; k++)
                    {
                        set = Subset[j];
                        set.erase(std::remove(set.begin(), set.end(), Subset[j][k]), set.end());
                        auto it = std::find(Subset.begin(), Subset.end(), set);
                        index_ = std::distance(Subset.begin(), it);

                        if (dp_re[i][j] != min(dp_re[i][j] , distant_mat[i][Subset[j][k]]+dp_re[Subset[j][k]][index_]))
                        {                            
                            dp_re[i][j] = distant_mat[i][Subset[j][k]]+dp_re[Subset[j][k]][index_];
                            pre[i][j]=Subset[j][k];
                        }                        
                    }
                }
            }
        }
        int lastrow = pow(2,waypoint_num-1)-1;
        dp_re[0][lastrow] =INT16_MAX;
        for(int k=0;k<Subset.back().size();k++)  //最后一个子集的每一个元素
        {
            set = Subset.back(); //最后一个子集
            set.erase(std::remove(set.begin(), set.end(), set[k]), set.end());
            auto it = std::find(Subset.begin(), Subset.end(), set);
            index_ = std::distance(Subset.begin(), it);

            if (dp_re[0][lastrow] != min(dp_re[0][lastrow] , distant_mat[0][Subset[lastrow][k]]+dp_re[Subset[lastrow][k]][index_]))
            {
                dp_re[0][lastrow] = distant_mat[0][Subset[lastrow][k]]+dp_re[Subset[lastrow][k]][index_] ;
                pre[0][lastrow] = Subset[lastrow][k];
            }
        }
        // cout  << "0  " << lastrow << "  "<< dp_re[0][lastrow] << ";  "<< "pre: "<<pre[0][lastrow]  <<endl;
        cout << "最短路径长度 " << dp_re[0][lastrow] << endl;
        std::vector<int> currentset = Subset[lastrow]; //最后一个子集作为当前子集，用于回溯路径
        deque<int> waypoint_flag;
        int _pre_,lastpre_=0;
        while(!currentset.empty())
        {
            auto it = std::find(Subset.begin(), Subset.end(), currentset);
            int index = std::distance(Subset.begin(), it);

            _pre_ = pre[lastpre_][index];
            waypoint_flag.push_back(_pre_);  //保存前驱
            lastpre_ = _pre_ ;
            // cout << _pre_ << "  " ;
            currentset.erase(std::remove(currentset.begin(), currentset.end(), _pre_), currentset.end());  //删除前驱
        }
        // /*排列视点*/
        std::vector<Eigen::Vector3d> result_re;
        for (const Eigen::Vector3d& element : reviewpoint) {
            result_re.push_back(element);
        }
        for (const int& element : waypoint_flag) {
            result.push_back(result_re[element]);
        }  //列表转化成向量
        
        // for (const Eigen::Vector3d& element : result) {
        //     cout << element[0] << " " << element[1] << " " << element[2] << endl;
        // } 
        return result;
    }
    list<Eigen::Vector3d> calculate_sort_viewpoint_greedy(list<Eigen::Vector3d> reviewpoint, bool is_fixed_lastpoint)  //排列后的视点顺序路径点
    {
        list<Eigen::Vector3d> result;
        /*获得一个距离矩阵*/
        std::vector<std::vector<double>> distant_mat;
        distant_mat = calculate_distant_mat (reviewpoint); 
        int waypoint_num =  distant_mat.size();  //路径点个数
        std::vector<int> waypoint_flag;
        for (int i=0 ; i<distant_mat.size() ; i++)  distant_mat[i][0] = INT16_MAX;      //起点出发，第一列赋值为无穷大
        /*打印距离矩阵*/
        // for (int m=0 ; m<distant_mat.size();m++)
        // {
        //     for (int n=0 ; n<distant_mat[i].size() ; n++)
        //         cout << distant_mat[m][n] << "  ";
        //     cout << endl;
        // }
        int index=0,dot=0;
        for (int i=0 ; i< waypoint_num-1 ; i++)  //从起点出发，找waypoint_num-1次最近点
        {
            auto minElementIterator = std::min_element(distant_mat[dot].begin(), distant_mat[dot].end());
            dot = std::distance(distant_mat[dot].begin(), minElementIterator);
            for (int j=0 ; j<distant_mat.size() ; j++)  distant_mat[j][dot] = INT16_MAX;
            waypoint_flag.push_back(dot); //保存路径点下标
            index = dot;
            // cout << "dot:"<<dot<<";"<<endl;
        }
        // /*排列视点*/
        std::vector<Eigen::Vector3d> result_re;
        for (const Eigen::Vector3d& element : reviewpoint) {
            result_re.push_back(element);
        }
        for (const int& element : waypoint_flag) {
            result.push_back(result_re[element]);
        }  //列表转化成向量
        return result;
    }
};

////////////
float filter_para = 2;        //滤波器参数 0.7
double normal_radius = 8.0;     //所有点云法向量搜索半径参数 8.0
std::string normalbox = "/home/nuc/ws_caric/src/vision_create/config/box_description.yaml";
float Fov_Really_size =0;   //视点的最大索引数值
float fov_interst[9000][6];//装视点的容器
double fov_horizontal_deg = 60.0; // 水平FOV（度）
double fov_vertical_deg = 45.0;   // 垂直FOV（度）
double fov_distance =2;          //FOV的长度
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
void selectViewpoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr clouds,pcl::PointCloud<pcl::Normal>::Ptr normals,pcl::PointCloud<pcl::PointXYZ>::Ptr& viewpoints,pcl::PointCloud<pcl::Normal>::Ptr& viewpointNormals) 
{
    int cover_size = 0;
    //创建KdTree对象
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(clouds);
    //创建一个临时点云用于修改
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = *clouds;
    int num = 364;
    // 索引0点的坐标
    pcl::PointXYZ search_point = cloud->points[num];
    //之前的最大点索引
    std::set<int> usedPointIndices;
    // 定义搜索半径
    float radius = 3; 
    //定义第二次搜索的范围（初始半径的2.5倍）
    float secondSearchRadius = 2.5 * radius;
    
    for(int j=0;j<1200;j++)
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
        cover_size+=firstPointIndices.size();
        //搜索该点2.5倍半径内的点
        std::vector<int> secondPointIndices;
        std::vector<float> secondPointDistances;
        kdtree.radiusSearch(search_point, secondSearchRadius, secondPointIndices, secondPointDistances);//2.5
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
            //if (std::find(firstPointIndices.begin(), firstPointIndices.end(), currentPointIndex) == firstPointIndices.end())
           // {
                // 计算第二次搜索半径内的点
                pcl::PointXYZ currentPoint = cloud->points[currentPointIndex];
                // 创建一个用于第二次搜索的新KdTree
                pcl::KdTreeFLANN<pcl::PointXYZ> secondKdTree;
                secondKdTree.setInputCloud(cloud);
                // 在第二次搜索半径内执行半径搜索
                std::vector<int> secondNeighborIndices;
                std::vector<float> secondNeighborDistances;
                secondKdTree.radiusSearch(currentPoint, radius, secondNeighborIndices, secondNeighborDistances);
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
            //}
        }
        // std::cout << "search_point after iteration " << j << ": (" << search_point.x << ", " << search_point.y << ", " << search_point.z << ")" << std::endl;
        search_point = maxNeighborPoint; 
        //记录所有fov索引        
        if(cover_size>=cloud->size())
            break;
        else if(maxPointIndex==0)
        {
            // 创建一个随机数生成器
            std::random_device rd;
            std::mt19937 generator(rd());
            std::uniform_int_distribution<int> distribution(1, cloud->size());
            int randomValue;
            do 
            {
                randomValue = distribution(generator);
            } 
            while (usedPointIndices.find(randomValue) != usedPointIndices.end());
            search_point = cloud->points[randomValue];
            continue;
        }
        FOVPointIndices.insert(maxPointIndex);   
        // std::cout << "currentPointIndex after iteration " << j << ": " << maxPointIndex << std::endl;
    }

    std::cout << "cover_size: " << cover_size << std::endl;
    // return FOVPointIndices;
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
struct BoxInfo {
    Eigen::Vector3f center;
    Eigen::Vector3f size;
    Eigen::Matrix4f orientation;};
bool PointIsInsideBox(const pcl::PointXYZ& point, const BoxInfo& box) 
{
    // 将点从全局坐标系转换到盒子的局部坐标系中
    Eigen::Vector3f local_point(
        point.x - box.center[0],
        point.y - box.center[1],
        point.z - box.center[2]
    );

    // 将点根据盒子的方向矩阵旋转到局部坐标系
    Eigen::Vector3f local_point_rotated = box.orientation.block<3, 3>(0, 0).transpose() * local_point;

    // 检查点是否在盒子的长方形范围内
    Eigen::Vector3f half_size = 0.5f * box.size;
    return (
        std::abs(local_point_rotated[0]) <= half_size[0] &&
        std::abs(local_point_rotated[1]) <= half_size[1] &&
        std::abs(local_point_rotated[2]) <= half_size[2]
    );
}
//校正法向量 使用box校正
void BoxCorrectNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals) 
{
    // 创建一个容器来存储多个BoxInfo结构
    std::vector<BoxInfo> boxInfos;
    // 填充box1的数据
    BoxInfo box1;// 待优化
    box1.center = Eigen::Vector3f(38.86922455, -0.34078693, 55.82170486);
    box1.size = Eigen::Vector3f(115.09100342, 25.26037979, 12.36612701);
    box1.orientation << 0.909586131573, 0.415515899658, 0.0, 38.869224548340,
                       -0.415515899658, 0.909586131573, 0.0, -0.340786933899,
                       0.0, 0.0, 1.0, 55.821704864502,
                       0.0, 0.0, 0.0, 1.0;
    boxInfos.push_back(box1);
    // 填充box2的数据
    BoxInfo box2;
    box2.center = Eigen::Vector3f(13.15921211, 12.35749626, 30.58357239);
    box2.size = Eigen::Vector3f(27.01351166, 15.44753742, 50.22055817);
    box2.orientation << 0.966737091541, 0.255773037672, 0.0, 13.159212112427,
                       -0.255773037672, 0.966737091541, 0.0, 12.357496261597,
                       0.0, 0.0, 1.0, 30.583572387695,
                       0.0, 0.0, 0.0, 1.0;
    boxInfos.push_back(box2);
    // 填充box3的数据
    BoxInfo box3;
    box3.center = Eigen::Vector3f(43.73695755, 3.39359474, 29.92310524);
    box3.size = Eigen::Vector3f(24.54467773, 19.53195190, 46.30308151);
    box3.orientation << 0.898045241833, 0.439902931452, 0.0, 43.736961364746,
                       -0.439902931452, 0.898045241833, 0.0, 3.393593788147,
                       0.0, 0.0, 1.0, 29.770519256592,
                       0.0, 0.0, 0.0, 1.0;
    boxInfos.push_back(box3);
    // 填充box4的数据
    BoxInfo box4;
    box4.center = Eigen::Vector3f(70.28045654, -12.58303833, 30.04368019);
    box4.size = Eigen::Vector3f(25.38685608, 14.20747375, 46.55189896);
    box4.orientation << 0.829285323620, 0.558830618858, 0.0, 70.280456542969,
                       -0.558830618858, 0.829285323620, 0.0, -12.583038330078,
                       0.0, 0.0, 1.0, 30.043680191040,
                       0.0, 0.0, 0.0, 1.0;
    boxInfos.push_back(box4);
    // 对每个盒子执行法向量校正
    for (int i = 0; i < cloud->size(); ++i) 
    {
        pcl::PointXYZ point = cloud->points[i];
        pcl::Normal point_normal = cloud_normals->points[i];

        for (const BoxInfo& box : boxInfos) {
            if (PointIsInsideBox(point, box)) 
            {
                // Get the box's normal vector
                Eigen::Vector3f box_normal = box.orientation.block<3, 1>(0, 2);

                // Project the point's normal vector onto the box's orientation
                Eigen::Vector3f projected_normal = box.orientation.block<3, 3>(0, 0) * point_normal.getNormalVector3fMap();

                // If the dot product between the projected normal and the vector to center is negative, flip the normal
                if (projected_normal.dot(point.getVector3fMap() - box.center) < 0) 
                {
                    point_normal.getNormalVector3fMap() *= -1;
                }
                point_normal.getNormalVector3fMap() *= -1;
                cloud_normals->points[i] = point_normal;

                break;  // If the point is inside one box, no need to check other boxes
            }
        }
    }
}
//降采样
void downsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filter_para,filter_para,filter_para); // 设置降采样的体素大小
    sor.filter(*cloud);
}
//计算法向量
void computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(normal_radius); // 设置搜索半径，根据你的数据调整
    ne.compute(*cloud_normals);
}
void filter_ground(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // 创建分割对象
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        // 设置要拟合的模型（例如，一个平面）
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(1);  // 根据需要调整此阈值

        // 分割最大的平面组件（地面）
        seg.setInputCloud(cloud); // 如果适用，使用下采样后的云
        seg.segment(*inliers, *coefficients);

        // 提取地面点
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);  // 如果适用，使用下采样后的云
        extract.setIndices(inliers);
        extract.setNegative(true); // 将为真以提取非地面点
        extract.filter(*cloud);
}

// ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
// visualization_msgs::MarkerArray visual_able;
// marker_pub.publish(visual_able);
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) 
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    //计算测试物体的中心坐标
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid); //求质心
    pcl::PointXYZ Model_center(centroid[0], centroid[1], centroid[2]);//转换中心坐标格式
    //设置降采样滤波器
    downsamplePointCloud(cloud);
    filter_ground(cloud);
    //计算法向量
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    computeNormals(cloud, cloud_normals);
    //校正法向量方向
    //correctNormals(centroid,cloud,cloud_normals);
    BoxCorrectNormals(cloud,cloud_normals);
    std::cout << "法向量最大索引:" << cloud_normals->size() << std::endl;
    //根据策略筛选
    pcl::PointCloud<pcl::PointXYZ>::Ptr viewpoints; 
    pcl::PointCloud<pcl::Normal>::Ptr viewpointNormals;
    //根据法向量点云计算出视点
    Get_all_Fov(cloud,cloud_normals);
    //筛选
    auto start_time = std::chrono::high_resolution_clock::now();//开始计运行时间
    //
    selectViewpoint(cloud,cloud_normals,viewpoints,viewpointNormals);    

    auto end_time = std::chrono::high_resolution_clock::now();//结束计时
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "程序运行时间：" << duration.count() << " 毫秒" << std::endl;
    //可视化
    // pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    // viewer->setBackgroundColor (0, 0, 0);
    // viewer->addPointCloud<pcl::PointXYZ> (cloud,"sample cloud");//显示点云
    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");//设置点的大小
    // viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, cloud_normals, 1.5, 0.4, "normals");//显示法向量
    //viewer->addSphere(Model_center, 1.5, 1.0, 0.0, 0.0, "center_sphere");//显示物体中心
    // viewer->addSphere(pcl::PointXYZ(38.86922455,-0.34078693,55.82170486), 1.5, 0.0, 0.0, 1.0, "cen1");
    // viewer->addSphere(pcl::PointXYZ(13.15921211,12.35749626,30.58357239), 1.5, 0.0, 0.0, 1.0, "cen2");
    // viewer->addSphere(pcl::PointXYZ(43.73695755,03.39359474,29.92310524), 1.5, 0.0, 0.0, 1.0, "cen3");
    // viewer->addSphere(pcl::PointXYZ(70.28045654,-12.58303833, 30.04368019), 1.5, 0.0, 0.0, 1.0, "cen4");

    // Show_fov_Index(viewer,1);
    // Show_fov(viewer);//根据视点打印所有fov
    // viewer->addCoordinateSystem (1.0);
    // viewer->initCameraParameters ();
    // while (!viewer->wasStopped ())
    // {
    //     viewer->spinOnce (100);
    //     std::this_thread::sleep_for(100ms);
    // }
    /************************************TSP********************************************/
    TSP_Slover tsp_cal;
    list<Eigen::Vector3d> viewpoint;
    Eigen::Vector3d point;
    cout << "FOVPointIndices  " <<FOVPointIndices.size() <<  endl;
    for (const int& num : FOVPointIndices)
    {
        point[0] = cloud ->points[num].x;
        point[1] = cloud ->points[num].y;
        point[2] = cloud ->points[num].z;
        cout << "way: " << point[0]<<"  " << point[1] << "  " <<  point[2] <<endl;
        viewpoint.push_back(point);
    }
    Eigen::Vector3d agent = {0,0,0};
    
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);

    start_time = std::chrono::high_resolution_clock::now();
    tsp_cal = TSP_Slover(agent,viewpoint,2);
    end_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    cout<< "TSP time : " << duration.count() << " ms ;" <<endl;
    
    
    auto waypoint_cal = tsp_cal.get_waypoint();
    for (const Eigen::Vector3d& point_ : waypoint_cal)
    {
        cout << "waypoint: " << point_[0]<<"  " << point_[1] << "  " <<  point_[2] <<endl;
    }
    visualization_msgs::MarkerArray visual_able;
    visual_able = tsp_cal.visualization_msg(waypoint_cal);
    marker_pub.publish(visual_able);


    /**********************************************************************************************/

    

    while(1)
    {
         ros::Duration(0.25).sleep();
        marker_pub.publish(visual_able);
    }
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "point_cloud_processing_node");
    ros::NodeHandle nh_;
    /*tsp_test*/
    Eigen::Vector3d agentpose = {0,0,0};
    // Eigen::Vector3d n1 = {1,0,0};
    // Eigen::Vector3d n2 = {0,2,0};
    // Eigen::Vector3d n3 = {2,1,0};
    // Eigen::Vector3d n4 = {1,0,1};
    // Eigen::Vector3d n5 = {0,0,4};
    // list<Eigen::Vector3d> viewpoint;
    // viewpoint.push_back(n1);
    // viewpoint.push_back(n2);
    // viewpoint.push_back(n3);
    // viewpoint.push_back(n4);
    // viewpoint.push_back(n5);

    // TSP_Slover tsp_test;
    // tsp_test = TSP_Slover(agentpose,viewpoint,2);
    // auto waypoint_cal = tsp_test.get_waypoint();
    // for (const Eigen::Vector3d& point : waypoint_cal)
    // {
    //     cout << "waypoint: " << point[0]<<"  " << point[1] << "  " <<  point[2] <<endl;
    // }
    ros::Subscriber sub = nh_.subscribe<sensor_msgs::PointCloud2>("/pcl_output", 1, pointCloudCallback);
    
    ros::spin();
    return 0;
}
//在移植进入比赛的模型中时 要给定bonund box的中心点 对法向量进行重定向 否则FOV会算错方向 在测试时使用质心代替