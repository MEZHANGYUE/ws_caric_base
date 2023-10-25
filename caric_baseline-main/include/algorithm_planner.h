#include <Eigen/Dense>
#include <limits>
#include <list>
#include <algorithm> 
#include <deque>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
/*
    输入：智能体当前位置，需要到达的路径点位置列表，TSP求解方式     1:动态规划 ,  2:贪婪算法 ; 
    排序路径点，可视化。
*/
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

/*
    输入n个边界框 std::vector<Eigen::Vector3d>& box_points :   n个边界框，8*n个点；
    可以得到每个边界框的覆盖搜索路径 list<std::vector<Eigen::Vector3d>> path_point_set
*/
class gen_path
{
    public:
    gen_path(){};
    gen_path(std::vector<Eigen::Vector3d>& box_points ) // n个边界框，8*n个点；
    {
       cal_exploer_path(box_points);
    }
    list<std::vector<Eigen::Vector3d>>  get_path_point_set()
    {
        return path_point_set;
    }

    visualization_msgs::MarkerArray visual_generate () 
    {
        visualization_msgs::MarkerArray get_visual_generate;
        visualization_msgs::Marker visual_sub;
        geometry_msgs::Point pose;
        visual_sub.header.frame_id = "world";
        visual_sub.header.stamp = ros::Time();
        visual_sub.ns = "my_namespace";
        visual_sub.id = 0 ;
        visual_sub.action = visualization_msgs::Marker::ADD;
        visual_sub.pose.orientation.w = 1.0;
        visual_sub.scale.x = 0.2;
        visual_sub.scale.y = 0.2;
        visual_sub.scale.z = 0.2;
        visual_sub.color.a = 1.0; // Don't forget to set the alpha!
        visual_sub.color.r = 1.0;
        visual_sub.color.g = 0.0;
        visual_sub.color.b = 0.0;
        visual_sub.type = visualization_msgs::Marker::LINE_STRIP;
        for(const std::vector<Eigen::Vector3d>menber : path_point_set) //每一个边界框中的路径
        {
            for( const Eigen::Vector3d&p : menber)
            {
                pose.x = p[0];
                pose.y = p[1];
                pose.z = p[2];
                visual_sub.points.push_back(pose);
            }
            get_visual_generate.markers.push_back(visual_sub);
            visual_sub.points.clear();
            visual_sub.id ++;
        }
        return get_visual_generate;
    }

    private:
    Eigen::Vector3d singlebox_first_point ;
    list<std::vector<Eigen::Vector3d>> path_point_set ; //每一个边界框的探索路径点
    list<std::vector<Eigen::Vector3d>> box_points_analysis_res; //储存边界框点分配、排列结果，列表每一个元素表示一个边界框，有几个元素就有几个边界框
    int distant_trans = 2 ;  //定义覆盖路径上升的距离
    int boxdex = 0 ;
    std::vector<Eigen::Vector3d> analysis_single_box(std::vector<Eigen::Vector3d> single_box_points) //对单个边界框的点进行排列
    {
        std::vector<Eigen::Vector3d> single_box_points_ = single_box_points;
        std::vector<Eigen::Vector3d> res;
        /*按照每个点到第一个点的距离进行排列*/
        for(int i=1 ; i<7 ; i++) //遍历确定后面每一个位置的点
        {
            for(int j=i+1 ; j<8 ; j++) //遍历需要确定点的后面的所有点进行比较 交换位置
            {
                if ((single_box_points_[j] - single_box_points_[0]).norm() < (single_box_points_[i] - single_box_points_[0]).norm())
                std::swap(single_box_points_[i], single_box_points_[j]);    
            }
        }
        std::swap(single_box_points_[2], single_box_points_[3]);  //交换前四个点，使前四个点按照顺时针或者逆时针排列
        res = single_box_points_;
        for(int i =4 ; i<8 ; i++)//排列后四个点对应前四个点
        {
            for(int j =4 ; j<8 ; j++)  //排列后四个点与第i个点的距离进行比较 交换位置
            {
                if((res[i]-res[i%4]).norm() > (single_box_points_[j]-res[i%4]).norm())      res[i]=single_box_points_[j]; 
            }
        }
        return res;
    }

    list<std::vector<Eigen::Vector3d>> analysis_box(std::vector<Eigen::Vector3d>& box_points)  //解析边界框的点，列表中每一个元素表示一个边界框
    {
        list<std::vector<Eigen::Vector3d>> res_,res;
        std::vector<Eigen::Vector3d> box_;
        if(box_points.size() %8 !=0) 
        {
            cout << "the box points size is fault in analysis !" <<endl;
            return res;
        }
        box_.clear();
        for (int i=0 ; i<=box_points.size() ; i++) //将边界框的点分配到列表中
        {
            if(i%8 == 0 && !box_.empty()) 
            {
                res_.push_back(box_);
                box_.clear();
            }
            if (i<box_points.size()) box_.push_back(box_points[i]);             
        }
        for ( const std::vector<Eigen::Vector3d>& member : res_)  // 将每一个边界框的点按照顺便排列好
        {
            res.push_back (analysis_single_box(member)) ;
        }
        return res;
    }
    bool pointInsidePolygon(const Eigen::Vector3d& point,const Eigen::Vector3d& center, const Eigen::Vector3d& vertex1, const Eigen::Vector3d& vertex2, const Eigen::Vector3d& vertex3, const Eigen::Vector3d& vertex4) 
    {
        // 计算面的法线向量并单位化
        Eigen::Vector3d normal = (vertex2 - vertex1).cross(vertex4 - vertex1).normalized();
        // 计算射线方向
        Eigen::Vector3d rayDirection = (center - point ).normalized();
        // 检查射线是否与面平行
        double dotProduct = rayDirection.dot(normal);  //一个方向在另一个方向上的投影长度
        if (std::abs(dotProduct) < 1e-6) {
            return false;  // 射线平行于面，点不在内部
        }
        // 计算射线与面的交点
        double t = normal.dot(vertex1 - point) / dotProduct;
        // 检查交点是否在射线的正方向上
        if (t < 0) {
            return false;  // 交点在射线背后，点不在内部
        }
        // 计算交点的坐标
        Eigen::Vector3d intersectionPoint = point + t * rayDirection;
        // 检查交点是否在面的内部
        // 使用点积法检查交点是否在多边形的内部
        Eigen::Vector3d edge1 = vertex2 - vertex1;
        Eigen::Vector3d edge2 = vertex3 - vertex2;
        Eigen::Vector3d edge3 = vertex4 - vertex3;
        Eigen::Vector3d edge4 = vertex1 - vertex4;
        Eigen::Vector3d v1 = intersectionPoint - vertex1;
        Eigen::Vector3d v2 = intersectionPoint - vertex2;
        Eigen::Vector3d v3 = intersectionPoint - vertex3;
        Eigen::Vector3d v4 = intersectionPoint - vertex4;

        bool inside = (edge1.cross(v1).dot(normal) >= 0) &&
                    (edge2.cross(v2).dot(normal) >= 0) &&
                    (edge3.cross(v3).dot(normal) >= 0) &&
                    (edge4.cross(v4).dot(normal) >= 0);

        return inside;
    }
    
    bool is_inbox(int boxdex_ , Eigen::Vector3d exp_point) //判断一个点是否在任意一个边界框内部
    {
        Eigen::Vector3d center;
        int flag=0;
        int x_max=0,y_max=0,z_max=0;
        int x_min=INT16_MAX,y_min=INT16_MAX,z_min = INT16_MAX;
        int dex_= 0;
        for(const std::vector<Eigen::Vector3d>& single_box : box_points_analysis_res) //每一个边界框
        {
            if (dex_ == boxdex_) {dex_++; continue;}    //所在边界框不进行内部检查
            else     //当前点是否在其他边界框内部
           {    
                dex_++;
                center =( single_box[0] + single_box[6])/2;  //表示一个内部的点
                //输入点与边界框内部点连接的射线与边界框6个面的交点数量；
                if (pointInsidePolygon(exp_point,center,single_box[0], single_box[1],single_box[2],single_box[3])) ++flag;
                if (pointInsidePolygon(exp_point,center,single_box[4], single_box[5],single_box[6],single_box[7])) ++flag;
                if (pointInsidePolygon(exp_point,center,single_box[1], single_box[2],single_box[6],single_box[5])) ++flag;
                if (pointInsidePolygon(exp_point,center,single_box[0], single_box[3],single_box[7],single_box[4])) ++flag;
                if (pointInsidePolygon(exp_point,center,single_box[0], single_box[1],single_box[5],single_box[4])) ++flag;
                if (pointInsidePolygon(exp_point,center,single_box[3], single_box[2],single_box[6],single_box[7])) ++flag;
                if (flag == 1)
                {
                    flag = 0;return true;
                } 
                else flag =0;
            }
        }
        return false;
    }
    std::vector<Eigen::Vector3d> get_path_point (int boxdex_ , std::vector<Eigen::Vector3d> box_point)  //获取探索者的路径点
    {
        std::vector<Eigen::Vector3d> res;
        Eigen::Vector3d dot;
        auto hight = (box_point[0] - box_point[4]).norm(); //计算边界框长边的高度
        double distant_scale = distant_trans/hight ;
        double scale_sum=distant_scale;
        res.push_back(box_point[0]);  //第一个点作为起点
        for(int i=1 ; scale_sum+distant_scale<=1 ; i++)
        {
            scale_sum = i*distant_scale;
            dot = (1-scale_sum)*box_point[i%4] + scale_sum *box_point[i%4+4];  //线性差值
            if (!is_inbox(boxdex_,dot)) 
                res.push_back(dot);
        }
        return res;
    }

    void cal_exploer_path(std::vector<Eigen::Vector3d>& box_points)
    {
        
        box_points_analysis_res = analysis_box(box_points);
        boxdex=0;
        for (const std::vector<Eigen::Vector3d>& member : box_points_analysis_res)//生成边界框的路径
        {
            path_point_set.push_back(get_path_point(boxdex,member)); //boxdex : 第几个边界框的路径
            boxdex++;
        }
    }
};

class grid_map
{
    public:
    grid_map(){};
    grid_map()
    {
        
    }
    private:
};


