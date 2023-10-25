#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/MarkerArray.h>

#include "Eigen/Dense"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "Astar.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "utility.h"
#include <mutex>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <caric_mission/CreatePPComTopic.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
struct position_info{
    Eigen::Vector3d position;
    bool update=false;
};

class Boundingbox
{
public:

    Boundingbox()
    {
        center = Eigen::Vector3d(0, 0, 0);
        volume = 0;
        id = -1;
    };

    Boundingbox(string str)
    {
        vector<string> spilited_str;
        std::istringstream iss(str);
        std::string substring;
        while (std::getline(iss, substring, ','))
        {
            spilited_str.push_back(substring);
        }
        
        int i = 0;
        while (i < 24)
        {
            vertice.push_back(Eigen::Vector3d(stod(spilited_str[i]), stod(spilited_str[i + 1]), stod(spilited_str[i + 2])));
            i = i + 3;
        }
        
        while (i < 33)
        {
            rotation_matrix(i - 24) = stod(spilited_str[i]);
            i++;
        }
        while (i < 36)
        {
            center = Eigen::Vector3d(stod(spilited_str[i]), stod(spilited_str[i + 1]), stod(spilited_str[i + 2]));
            i = i + 3;
        }
        while (i < 39)
        {
            size_vector = Eigen::Vector3d(stod(spilited_str[i]), stod(spilited_str[i + 1]), stod(spilited_str[i + 2]));
            i = i + 3;
        }
        while (i < 42)
        {
            global_in_out.push_back(Eigen::Vector3d(stod(spilited_str[i]), stod(spilited_str[i + 1]), stod(spilited_str[i + 2])));
            i = i + 3;
        }
        while (i < 45)
        {
            global_in_out.push_back(Eigen::Vector3d(stod(spilited_str[i]), stod(spilited_str[i + 1]), stod(spilited_str[i + 2])));
            i = i + 3;
        }
        
        xsize = stod(spilited_str[i]);
        i++;
        ysize = stod(spilited_str[i]);
        i++;
        zsize = stod(spilited_str[i]);
        i++;
        id = stod(spilited_str[i]);
        i++;
        state = stod(spilited_str[i]);
        i++;
        volume = stod(spilited_str[i]);
        i++;
        use_x = stod(spilited_str[i]);
        i++;
        use_y = stod(spilited_str[i]);
        i++;
        use_z = stod(spilited_str[i]);
        i++;
    }

    Boundingbox(const std::vector<Eigen::Vector3d> &vec, int id_in, Eigen::Vector3d &start, Eigen::Vector3d &end, int state_in, bool x, bool y, bool z)
    {
        vertice = vec;
        id = id_in;
        center = Eigen::Vector3d(0, 0, 0);
        for (int index = 0; index < vec.size(); index++)
        {
            center = center + vec[index];
        }
        center = center / 8;
        xsize = (vec[1] - vec[0]).norm();
        ysize = (vec[3] - vec[0]).norm();
        zsize = (vec[4] - vec[0]).norm();
        volume = xsize * ysize * zsize;
        size_vector = Eigen::Vector3d(xsize / 2, ysize / 2, zsize / 2);
        Eigen::Vector3d xaxis, yaxis, zaxis;
        xaxis = (vec[1] - vec[0]).normalized();
        yaxis = (vec[3] - vec[0]).normalized();
        zaxis = (vec[4] - vec[0]).normalized();
        rotation_matrix << xaxis, yaxis, zaxis;
        global_in_out.push_back(start);
        global_in_out.push_back(end);
        state = state_in;
        use_x = x;
        use_y = y;
        use_z = z;
    }
    
    Boundingbox(const std::vector<Eigen::Vector3d> &vec, int id_in)
    {
        vertice = vec;
        id = id_in;
        center = Eigen::Vector3d(0, 0, 0);
        for (int index = 0; index < vec.size(); index++)
        {
            center = center + vec[index];
        }
        center = center / 8;
        xsize = (vec[1] - vec[0]).norm();
        ysize = (vec[3] - vec[0]).norm();
        zsize = (vec[4] - vec[0]).norm();

        volume = xsize * ysize * zsize;
        size_vector = Eigen::Vector3d(xsize / 2, ysize / 2, zsize / 2);

        Eigen::Vector3d xaxis, yaxis, zaxis;
        xaxis = (vec[1] - vec[0]).normalized();
        yaxis = (vec[3] - vec[0]).normalized();
        zaxis = (vec[4] - vec[0]).normalized();

        Eigen::Vector3d xplus, xminus, yplus, yminus, zplus, zminus;
        yminus = (vec[0] + vec[1] + vec[4] + vec[5]) / 4;
        yplus = (vec[2] + vec[3] + vec[6] + vec[7]) / 4;

        xminus = (vec[0] + vec[3] + vec[4] + vec[7]) / 4;
        xplus = (vec[1] + vec[2] + vec[5] + vec[6]) / 4;

        zminus = (vec[0] + vec[1] + vec[2] + vec[3]) / 4;
        zplus = (vec[4] + vec[5] + vec[6] + vec[7]) / 4;

        rotation_matrix << xaxis, yaxis, zaxis;
        if ((zsize >= ysize) && (zsize >= xsize))
        {
            use_z = true;
            global_in_out.push_back(zminus);
            global_in_out.push_back(zplus);
        }
        else if ((xsize >= ysize) && (xsize >= zsize))
        {
            if (global_in_out.size() == 0)
            {
                use_x = true;
                global_in_out.push_back(xminus);
                global_in_out.push_back(xplus);
            }
        }
        else if ((ysize >= xsize) && (ysize >= zsize))
        {
            if (global_in_out.size() == 0)
            {
                use_y = true;
                global_in_out.push_back(yminus);
                global_in_out.push_back(yplus);
            }
        }
        else
        {
            if (global_in_out.size() == 0)
            {
                use_z = true;
                global_in_out.push_back(zminus);
                global_in_out.push_back(zplus);
            }
        }
    };

    ~Boundingbox(){};
    
    const Matrix3d getSearchRotation() const
    {
        Eigen::Vector3d axis_rotation_along(0.0, 0.0, 1.0);
        Eigen::Matrix3d transfer_matrix;
        Eigen::Matrix3d result;
        double angle = 0;
        if (use_x)
        {
            if (state == 0)
            {
                cout << "x+" << endl;
                axis_rotation_along = Eigen::Vector3d(0.0, 1.0, 0.0);
                angle = -M_PI / 2;
            }
            else if (state == 1)
            {
                cout << "x-" << endl;
                axis_rotation_along = Eigen::Vector3d(0.0, 1.0, 0.0);
                angle = M_PI / 2;
            }
            else
            {
                cout << "Error State" << endl;
            }
        }
        else if (use_y)
        {
            if (state == 0)
            {
                cout << "y+" << endl;
                axis_rotation_along = Eigen::Vector3d(1.0, 0.0, 0.0);
                angle = M_PI / 2;
            }
            else if (state == 1)
            {
                cout << "y-" << endl;
                axis_rotation_along = Eigen::Vector3d(1.0, 0.0, 0.0);
                angle = -M_PI / 2;
            }
            else
            {
                cout << "Error State" << endl;
            }
        }
        else if (use_z)
        {
            if (state == 0)
            {
                cout << "z+" << endl;
            }
            else if (state == 1)
            {
                cout << "z-" << endl;
                axis_rotation_along = Eigen::Vector3d(1.0, 0.0, 0.0);
                angle = M_PI;
            }
            else
            {
                cout << "Error State" << endl;
            }
        }
        else
        {
            cout << "noting happen" << endl;
        }

        transfer_matrix = Eigen::AngleAxisd(angle, axis_rotation_along);
        result = transfer_matrix * rotation_matrix.inverse();
        return result;
        
    }

    int getState()
    {
        return state;
    }

    double getVolume() const
    {
        return volume;
    }
    const Vector3d getCenter() const
    {
        return center;
    }
    const Matrix3d getRotation() const
    {
        return rotation_matrix;
    }
    const Vector3d getExtents() const
    {
        return size_vector;
    }
    const Vector3d getRotExtents() const
    {
        Eigen::Vector3d result(0, 0, 0);
        if (use_x)
        {
            if (state == 0)
            {
                // cout<<"x+"<<endl;
                result.x() = size_vector.z();
                result.y() = size_vector.y();
                result.z() = size_vector.x();
            }
            else if (state == 1)
            {
                result.x() = size_vector.z();
                result.y() = size_vector.y();
                result.z() = size_vector.x();
                // cout<<"x-"<<endl;
                // axis_rotation_along=Eigen::Vector3d(0.0,1.0,0.0);
                // angle=M_PI/2;
            }
            else
            {
                cout << "Error State" << endl;
            }
        }
        else if (use_y)
        {
            if (state == 0)
            {
                // cout<<"y+"<<endl;
                result.x() = size_vector.x();
                result.y() = size_vector.z();
                result.z() = size_vector.y();
            }
            else if (state == 1)
            {
                // cout<<"y-"<<endl;
                result.x() = size_vector.x();
                result.y() = size_vector.z();
                result.z() = size_vector.y();
            }
            else
            {
                cout << "Error State" << endl;
            }
        }
        else if (use_z)
        {
            if (state == 0)
            {
                // cout<<"z+"<<endl;
                result = size_vector;
            }
            else if (state == 1)
            {
                // cout<<"z-"<<endl;
                result = size_vector;
            }
            else
            {
                cout << "Error State" << endl;
            }
        }
        else
        {
            cout << "noting happen" << endl;
        }
        return result;
    }
    vector<Eigen::Vector3d> getVertices() const
    {
        return vertice;
    }
    Eigen::Vector3d get_global_in_out(int state) const
    {
        Eigen::Vector3d result = global_in_out[state];
        return result;
    }
    void edit_state(int state_in)
    {
        state = state_in;
    }
    void edit_id()
    {
        id = id + 1;
    }
    int getId()
    {
        return id;
    }

    void generate_start(double scale, Boundingbox &start, Boundingbox &end)
    {
        if (use_z)
        {
            vector<Eigen::Vector3d> new_vertice_start(8);
            vector<Eigen::Vector3d> new_vertice_end(8);
            if (state == 0)
            {
                new_vertice_start[0] = vertice[0];
                new_vertice_start[1] = vertice[1];
                new_vertice_start[2] = vertice[2];
                new_vertice_start[3] = vertice[3];
                new_vertice_start[4] = vertice[0] + (vertice[4] - vertice[0]) * scale;
                new_vertice_start[5] = vertice[1] + (vertice[5] - vertice[1]) * scale;
                new_vertice_start[6] = vertice[2] + (vertice[6] - vertice[2]) * scale;
                new_vertice_start[7] = vertice[3] + (vertice[7] - vertice[3]) * scale;

                new_vertice_end[0] = vertice[0] + (vertice[4] - vertice[0]) * scale;
                new_vertice_end[1] = vertice[1] + (vertice[5] - vertice[1]) * scale;
                new_vertice_end[2] = vertice[2] + (vertice[6] - vertice[2]) * scale;
                new_vertice_end[3] = vertice[3] + (vertice[7] - vertice[3]) * scale;

                new_vertice_end[4] = vertice[4];
                new_vertice_end[5] = vertice[5];
                new_vertice_end[6] = vertice[6];
                new_vertice_end[7] = vertice[7];

                Eigen::Vector3d new_start = (new_vertice_start[0] + new_vertice_start[1] + new_vertice_start[2] + new_vertice_start[3]) / 4;
                Eigen::Vector3d new_end = (new_vertice_start[4] + new_vertice_start[5] + new_vertice_start[6] + new_vertice_start[7]) / 4;
                Eigen::Vector3d end_start = (new_vertice_end[0] + new_vertice_end[1] + new_vertice_end[2] + new_vertice_end[3]) / 4;
                Eigen::Vector3d end_end = (new_vertice_end[4] + new_vertice_end[5] + new_vertice_end[6] + new_vertice_end[7]) / 4;

                start = Boundingbox(new_vertice_start, id, end_start, end_end, state, use_x, use_y, use_z);
                end = Boundingbox(new_vertice_end, id, end_start, end_end, state, use_x, use_y, use_z);
                return;
            }
            else
            {
                scale = 1 - scale;
                new_vertice_start[0] = vertice[0];
                new_vertice_start[1] = vertice[1];
                new_vertice_start[2] = vertice[2];
                new_vertice_start[3] = vertice[3];
                new_vertice_start[4] = vertice[0] + (vertice[4] - vertice[0]) * scale;
                new_vertice_start[5] = vertice[1] + (vertice[5] - vertice[1]) * scale;
                new_vertice_start[6] = vertice[2] + (vertice[6] - vertice[2]) * scale;
                new_vertice_start[7] = vertice[3] + (vertice[7] - vertice[3]) * scale;

                new_vertice_end[0] = vertice[0] + (vertice[4] - vertice[0]) * scale;
                new_vertice_end[1] = vertice[1] + (vertice[5] - vertice[1]) * scale;
                new_vertice_end[2] = vertice[2] + (vertice[6] - vertice[2]) * scale;
                new_vertice_end[3] = vertice[3] + (vertice[7] - vertice[3]) * scale;

                new_vertice_end[4] = vertice[4];
                new_vertice_end[5] = vertice[5];
                new_vertice_end[6] = vertice[6];
                new_vertice_end[7] = vertice[7];

                Eigen::Vector3d new_start = (new_vertice_start[0] + new_vertice_start[1] + new_vertice_start[2] + new_vertice_start[3]) / 4;
                Eigen::Vector3d new_end = (new_vertice_start[4] + new_vertice_start[5] + new_vertice_start[6] + new_vertice_start[7]) / 4;
                Eigen::Vector3d end_start = (new_vertice_end[0] + new_vertice_end[1] + new_vertice_end[2] + new_vertice_end[3]) / 4;
                Eigen::Vector3d end_end = (new_vertice_end[4] + new_vertice_end[5] + new_vertice_end[6] + new_vertice_end[7]) / 4;

                start = Boundingbox(new_vertice_end, id, end_start, end_end, state, use_x, use_y, use_z);
                end = Boundingbox(new_vertice_start, id, end_start, end_end, state, use_x, use_y, use_z);
                return;
            }
        }
        else if (use_x)
        {
            vector<Eigen::Vector3d> new_vertice_start(8);
            vector<Eigen::Vector3d> new_vertice_end(8);
            if (state == 0)
            {
                new_vertice_start[0] = vertice[0];
                new_vertice_start[3] = vertice[3];
                new_vertice_start[4] = vertice[4];
                new_vertice_start[7] = vertice[7];
                new_vertice_start[1] = vertice[0] + (vertice[1] - vertice[0]) * scale;
                new_vertice_start[2] = vertice[3] + (vertice[2] - vertice[3]) * scale;
                new_vertice_start[5] = vertice[4] + (vertice[5] - vertice[4]) * scale;
                new_vertice_start[6] = vertice[7] + (vertice[6] - vertice[7]) * scale;

                new_vertice_end[0] = vertice[0] + (vertice[1] - vertice[0]) * scale;
                new_vertice_end[3] = vertice[3] + (vertice[2] - vertice[3]) * scale;
                new_vertice_end[4] = vertice[4] + (vertice[5] - vertice[4]) * scale;
                new_vertice_end[7] = vertice[7] + (vertice[6] - vertice[7]) * scale;

                new_vertice_end[1] = vertice[1];
                new_vertice_end[2] = vertice[2];
                new_vertice_end[5] = vertice[5];
                new_vertice_end[6] = vertice[6];

                Eigen::Vector3d new_start = (new_vertice_start[0] + new_vertice_start[3] + new_vertice_start[4] + new_vertice_start[7]) / 4;
                Eigen::Vector3d new_end = (new_vertice_start[1] + new_vertice_start[2] + new_vertice_start[5] + new_vertice_start[6]) / 4;
                Eigen::Vector3d end_start = (new_vertice_end[0] + new_vertice_end[3] + new_vertice_end[4] + new_vertice_end[7]) / 4;
                Eigen::Vector3d end_end = (new_vertice_end[1] + new_vertice_end[2] + new_vertice_end[5] + new_vertice_end[6]) / 4;

                start = Boundingbox(new_vertice_start, id, end_start, end_end, state, use_x, use_y, use_z);
                end = Boundingbox(new_vertice_end, id, end_start, end_end, state, use_x, use_y, use_z);
                return;
            }
            else
            {
                scale = 1 - scale;
                new_vertice_start[0] = vertice[0];
                new_vertice_start[3] = vertice[3];
                new_vertice_start[4] = vertice[4];
                new_vertice_start[7] = vertice[7];
                new_vertice_start[1] = vertice[0] + (vertice[1] - vertice[0]) * scale;
                new_vertice_start[2] = vertice[3] + (vertice[2] - vertice[3]) * scale;
                new_vertice_start[5] = vertice[4] + (vertice[5] - vertice[4]) * scale;
                new_vertice_start[6] = vertice[7] + (vertice[6] - vertice[7]) * scale;

                new_vertice_end[0] = vertice[0] + (vertice[1] - vertice[0]) * scale;
                new_vertice_end[3] = vertice[3] + (vertice[2] - vertice[3]) * scale;
                new_vertice_end[4] = vertice[4] + (vertice[5] - vertice[4]) * scale;
                new_vertice_end[7] = vertice[7] + (vertice[6] - vertice[7]) * scale;

                new_vertice_end[1] = vertice[1];
                new_vertice_end[2] = vertice[2];
                new_vertice_end[5] = vertice[5];
                new_vertice_end[6] = vertice[6];

                Eigen::Vector3d new_start = (new_vertice_start[0] + new_vertice_start[3] + new_vertice_start[4] + new_vertice_start[7]) / 4;
                Eigen::Vector3d new_end = (new_vertice_start[1] + new_vertice_start[2] + new_vertice_start[5] + new_vertice_start[6]) / 4;
                Eigen::Vector3d end_start = (new_vertice_end[0] + new_vertice_end[3] + new_vertice_end[4] + new_vertice_end[7]) / 4;
                Eigen::Vector3d end_end = (new_vertice_end[1] + new_vertice_end[2] + new_vertice_end[5] + new_vertice_end[6]) / 4;

                start = Boundingbox(new_vertice_end, id, end_start, end_end, state, use_x, use_y, use_z);
                end = Boundingbox(new_vertice_start, id, end_start, end_end, state, use_x, use_y, use_z);
                return;
            }
        }
        else
        {
            vector<Eigen::Vector3d> new_vertice_start(8);
            vector<Eigen::Vector3d> new_vertice_end(8);
            if (state == 0)
            {
                new_vertice_start[0] = vertice[0];
                new_vertice_start[1] = vertice[1];
                new_vertice_start[5] = vertice[5];
                new_vertice_start[4] = vertice[4];
                new_vertice_start[3] = vertice[0] + (vertice[3] - vertice[0]) * scale;
                new_vertice_start[2] = vertice[1] + (vertice[2] - vertice[1]) * scale;
                new_vertice_start[6] = vertice[5] + (vertice[6] - vertice[5]) * scale;
                new_vertice_start[7] = vertice[4] + (vertice[7] - vertice[4]) * scale;

                new_vertice_end[0] = vertice[0] + (vertice[3] - vertice[0]) * scale;
                new_vertice_end[1] = vertice[1] + (vertice[2] - vertice[1]) * scale;
                new_vertice_end[5] = vertice[5] + (vertice[6] - vertice[5]) * scale;
                new_vertice_end[4] = vertice[4] + (vertice[7] - vertice[4]) * scale;

                new_vertice_end[3] = vertice[3];
                new_vertice_end[2] = vertice[2];
                new_vertice_end[6] = vertice[6];
                new_vertice_end[7] = vertice[7];

                Eigen::Vector3d new_start = (new_vertice_start[0] + new_vertice_start[1] + new_vertice_start[5] + new_vertice_start[4]) / 4;
                Eigen::Vector3d new_end = (new_vertice_start[3] + new_vertice_start[2] + new_vertice_start[6] + new_vertice_start[7]) / 4;
                Eigen::Vector3d end_start = (new_vertice_end[0] + new_vertice_end[1] + new_vertice_end[5] + new_vertice_end[4]) / 4;
                Eigen::Vector3d end_end = (new_vertice_end[3] + new_vertice_end[2] + new_vertice_end[6] + new_vertice_end[7]) / 4;

                start = Boundingbox(new_vertice_start, id, end_start, end_end, state, use_x, use_y, use_z);
                end = Boundingbox(new_vertice_end, id, end_start, end_end, state, use_x, use_y, use_z);
                return;
            }
            else
            {
                scale = 1 - scale;
                new_vertice_start[0] = vertice[0];
                new_vertice_start[1] = vertice[1];
                new_vertice_start[5] = vertice[5];
                new_vertice_start[4] = vertice[4];
                new_vertice_start[3] = vertice[0] + (vertice[3] - vertice[0]) * scale;
                new_vertice_start[2] = vertice[1] + (vertice[2] - vertice[1]) * scale;
                new_vertice_start[6] = vertice[5] + (vertice[6] - vertice[5]) * scale;
                new_vertice_start[7] = vertice[4] + (vertice[7] - vertice[4]) * scale;

                new_vertice_end[0] = vertice[0] + (vertice[3] - vertice[0]) * scale;
                new_vertice_end[1] = vertice[1] + (vertice[2] - vertice[1]) * scale;
                new_vertice_end[5] = vertice[5] + (vertice[6] - vertice[5]) * scale;
                new_vertice_end[4] = vertice[4] + (vertice[7] - vertice[4]) * scale;

                new_vertice_end[3] = vertice[3];
                new_vertice_end[2] = vertice[2];
                new_vertice_end[6] = vertice[6];
                new_vertice_end[7] = vertice[7];

                Eigen::Vector3d new_start = (new_vertice_start[0] + new_vertice_start[1] + new_vertice_start[5] + new_vertice_start[4]) / 4;
                Eigen::Vector3d new_end = (new_vertice_start[3] + new_vertice_start[2] + new_vertice_start[6] + new_vertice_start[7]) / 4;
                Eigen::Vector3d end_start = (new_vertice_end[0] + new_vertice_end[1] + new_vertice_end[5] + new_vertice_end[4]) / 4;
                Eigen::Vector3d end_end = (new_vertice_end[3] + new_vertice_end[2] + new_vertice_end[6] + new_vertice_end[7]) / 4;

                start = Boundingbox(new_vertice_end, id, end_start, end_end, state, use_x, use_y, use_z);
                end = Boundingbox(new_vertice_start, id, end_start, end_end, state, use_x, use_y, use_z);
                return;
            }
        }
    }

    string generate_string_version() const
    {
        string result = "";
        for (int i = 0; i < 8; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                result = result + to_string(vertice[i][j]) + ",";
            }
        }
        for (int i = 0; i < 9; i++)
        {
            result = result + to_string(rotation_matrix(i)) + ",";
        }
        for (int j = 0; j < 3; j++)
        {
            result = result + to_string(center[j]) + ",";
        }
        for (int j = 0; j < 3; j++)
        {
            result = result + to_string(size_vector[j]) + ",";
        }

        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                result = result + to_string(global_in_out[i][j]) + ",";
            }
        }
        result = result + to_string(xsize) + ",";
        result = result + to_string(ysize) + ",";
        result = result + to_string(zsize) + ",";
        result = result + to_string(id) + ",";
        result = result + to_string(state) + ",";
        result = result + to_string(volume) + ",";
        result = result + to_string(use_x) + ",";
        result = result + to_string(use_y) + ",";
        result = result + to_string(use_z) + ",";
        // cout<<result<<endl;
        return result;
    }
    
private:
    vector<Eigen::Vector3d> vertice; 
    double volume = 0;
    Eigen::Vector3d center;                
    Eigen::Matrix3d rotation_matrix;       
    Eigen::Vector3d size_vector;           
    double xsize, ysize, zsize;            
    int id = 0;                            
    vector<Eigen::Vector3d> global_in_out; 
    int state = 0;                         
    bool use_x = false;                    
    bool use_y = false;                    
    bool use_z = false;                    
};

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

class gcs_task_assign   
{
    public:
        gcs_task_assign(){

        }
        gcs_task_assign(ros::NodeHandlePtr &nh_ptr_)
        : nh_ptr_(nh_ptr_)
        {
            namelist = {"/jurong", "/raffles", "/changi", "/sentosa", "/nanyang"};
            position_pair["/jurong"] = {Eigen::Vector3d(0, 0, 1), false};
            position_pair["/raffles"] = {Eigen::Vector3d(0, 0, 1), false};
            position_pair["/changi"] = {Eigen::Vector3d(0, 0, 1), false};
            position_pair["/sentosa"] = {Eigen::Vector3d(0, 0, 1), false};
            position_pair["/nanyang"]={Eigen::Vector3d(0, 0, 1), false};

            xmax = -std::numeric_limits<double>::max();
            ymax = -std::numeric_limits<double>::max();
            zmax = -std::numeric_limits<double>::max();
            xmin = std::numeric_limits<double>::max();
            ymin = std::numeric_limits<double>::max();
            zmin = std::numeric_limits<double>::max();

            client = nh_ptr_->serviceClient<caric_mission::CreatePPComTopic>("create_ppcom_topic");
            cmd_pub_ = nh_ptr_->advertise<std_msgs::String>("/task_assign", 10);
            srv.request.source = "gcs";
            srv.request.targets.push_back("all");
            srv.request.topic_name = "/task_assign";
            srv.request.package_name = "std_msgs";
            srv.request.message_type = "String";

            while (!serviceAvailable)
            {
                serviceAvailable = ros::service::waitForService("create_ppcom_topic", ros::Duration(10.0));
            }
            client.call(srv);
            boxes_pub = nh_ptr_->advertise<visualization_msgs::MarkerArray>("/boxes_path", 10);  //发布沿着边界框的路径，用于探索者产生点云覆盖地图
            bbox_sub_ = nh_ptr_->subscribe<sensor_msgs::PointCloud>("/gcs/bounding_box_vertices", 10, &gcs_task_assign::bboxCallback, this);
            agent_position_sub_=nh_ptr_->subscribe<std_msgs::String>("/broadcast/gcs", 10, &gcs_task_assign::positionCallback, this);

            Agent_ensure_Timer=nh_ptr_->createTimer(ros::Duration(1.0 / 10.0),  &gcs_task_assign::TimerEnsureCB,     this);
            Massage_publish_Timer=nh_ptr_->createTimer(ros::Duration(1.0 / 10.0),  &gcs_task_assign::TimerMessageCB,     this);

            
            Gen_path = gen_path();
        }
    private:
        ros::NodeHandlePtr nh_ptr_;
        gen_path Gen_path;
        //communication related param
        bool serviceAvailable = false;
        caric_mission::CreatePPComTopic srv;
        ros::ServiceClient client;
        visualization_msgs::MarkerArray visual_boxpath;
        ros::Publisher cmd_pub_;
        ros::Publisher boxes_pub;
        ros::Subscriber agent_position_sub_;
        ros::Subscriber bbox_sub_;
        
        ros::Timer Agent_ensure_Timer;
        ros::Timer Massage_publish_Timer;

        sensor_msgs::PointCloud all_box_point;
        bool get_bbox=false;
        bool finish_massage_generate=false;
        bool agent_info_get=false;

        list<string> namelist;
        map<string,position_info> position_pair;
        double update_time;

        double volumn_total=0;
        bool finish_bbox_record = false;
        vector<Boundingbox> box_set;
        
        vector<int> box_index;
        vector<int> state_vec;


        double xmax;
        double ymax;
        double zmax;
        double xmin;
        double ymin;
        double zmin;

        vector<vector<string>> team_info;
        vector<vector<Boundingbox>> output_path;

        string result;
        geometry_msgs::Point32 pose;
        void bboxCallback(const sensor_msgs::PointCloud::ConstPtr &msg)
        {
            all_box_point.header.frame_id = "world"; 
            all_box_point.header.stamp = ros::Time();
            sensor_msgs::PointCloud cloud = *msg;
            int num_points = cloud.points.size();

            std::vector<Eigen::Vector3d> boxes_point_;
            Eigen::Vector3d pose;
            for (int i=0 ; i < msg->points.size(); i++)   //将边界框点云表示为向量形式
            {
                pose[0] = cloud.points[i].x ;
                pose[1] = cloud.points[i].y ;
                pose[2] = cloud.points[i].z ;
                boxes_point_.push_back(pose);       
            }
            Gen_path = gen_path(boxes_point_);  //std::vector<Eigen::Vector3d>
            visual_boxpath = Gen_path. visual_generate () ;
            boxes_pub.publish(visual_boxpath);  //发布沿着边界框的路径可视化

            if(finish_bbox_record||!agent_info_get)
            {
                return;
            }
            if(finish_massage_generate)
            {
                return;
            }

            if (num_points % 8 == 0 && num_points > 8 * box_set.size())
            {
                volumn_total = 0;
                int num_box = num_points / 8;
                for (int i = 0; i < num_box; i++)
                {
                    vector<Eigen::Vector3d> point_vec;
                    for (int j = 0; j < 8; j++)
                    {
                        if (cloud.points[8 * i + j].x > xmax)
                        {
                            xmax = cloud.points[8 * i + j].x;
                        }
                        if (cloud.points[8 * i + j].x < xmin)
                        {
                            xmin = cloud.points[8 * i + j].x;
                        }
                        if (cloud.points[8 * i + j].y > ymax)
                        {
                            ymax = cloud.points[8 * i + j].y;
                        }
                        if (cloud.points[8 * i + j].y < ymin)
                        {
                            ymin = cloud.points[8 * i + j].y;
                        }
                        if (cloud.points[8 * i + j].z > zmax)
                        {
                            zmax = cloud.points[8 * i + j].z;
                        }
                        if (cloud.points[8 * i + j].z < zmin)
                        {
                            zmin = cloud.points[8 * i + j].z;
                        }
                        point_vec.push_back(Eigen::Vector3d(cloud.points[8 * i + j].x, cloud.points[8 * i + j].y, cloud.points[8 * i + j].z));
                    }
                    box_set.push_back(Boundingbox(point_vec, i));
                    volumn_total += box_set[i].getVolume();
                    point_vec.clear();
                }
                finish_bbox_record = true;
                //Team allocate
                Team_allocate();
                //Use best first to generate the global trajactory
                Best_first_search();
                //Clip the best path
                Clip_the_task();
                //Generate the massage
                generate_massage();
            }
            else
            {
                return;
            }
            return;
        }

        void positionCallback(const std_msgs::String msg)
        {
            istringstream str(msg.data);
            string type;
            getline(str,type,';');
            if(type=="init_pos")
            {
                string origin;
                getline(str,origin,';');
                string position_str;
                getline(str,position_str,';');
                if(!position_pair[origin].update){
                    update_time=ros::Time::now().toSec();
                    position_pair[origin].position=str2point(position_str);
                    position_pair[origin].update=true;
                }
            }
        }

        void TimerEnsureCB(const ros::TimerEvent &)
        {   
            if(agent_info_get)
            {
                return;
            }
            bool finish_agent_record=false; 
            for(auto& name:namelist)
            {
                if(position_pair[name].update){
                    finish_agent_record=true;
                    break;
                }
            }
            if(!finish_agent_record)
            {
                return;
            }
            double time_now=ros::Time::now().toSec();

            if(fabs(time_now-update_time)>10)
            {
                agent_info_get=true;
            }
        }
        void TimerMessageCB(const ros::TimerEvent &)
        {
            if(finish_massage_generate)
            {
                std_msgs::String task;                
                task.data=result;
                cmd_pub_.publish(task);
            }
        }


        Eigen::Vector3d str2point(string input)
        {
            Eigen::Vector3d result;
            std::vector<string> value;
            boost::split(value, input, boost::is_any_of(","));
            // cout<<input<<endl;
            if (value.size() == 3)
            {
                result = Eigen::Vector3d(stod(value[0]), stod(value[1]), stod(value[2]));
            }
            else
            {
                cout << input << endl;
                cout << "error use str2point 2" << endl;
            }
            return result;
        }
        //分组
        void Team_allocate()
        {
            team_info=vector<vector<string>>(2);
            map<string,double> jurong_dis,raffles_dis;
            if(position_pair["/jurong"].update){
                team_info[0].push_back("/jurong");
            }
            if(position_pair["/raffles"].update){
                team_info[1].push_back("/raffles");
            }
            for(auto name:namelist){
                if(name=="/jurong"||name=="/raffles"){
                    continue;
                }
                if(!position_pair[name].update){
                    continue;
                }
                if(team_info[0].size()>0&&team_info[1].size()>0){
                    jurong_dis[name]=(position_pair[name].position-position_pair[team_info[0][0]].position).norm();
                    raffles_dis[name]=(position_pair[name].position-position_pair[team_info[1][0]].position).norm();
                    if(jurong_dis[name]<=raffles_dis[name]){
                        team_info[0].push_back(name);
                    }else{
                        team_info[1].push_back(name);
                    }
                    continue;
                }else if(team_info[0].size()>0){
                    team_info[0].push_back(name);
                    continue;
                }else if(team_info[1].size()>0){
                    team_info[1].push_back(name);
                    continue;
                }else{
                    continue;
                }
            }

            if(team_info[0].size()>0&&team_info[1].size()>0) //两个组都有成员，检查分配是否合理，
            {
                string transname=team_info[0][1]; //需要转移组队的无人机成员
                if(team_info[0].size()-team_info[1].size()>1)
                {
                    cout << "123" << endl;
                    while (team_info[0].size()-team_info[1].size()>1)
                    {
                        double dis_= raffles_dis[team_info[0][1]] - jurong_dis[team_info[0][1]];
                        for(auto teammate:team_info[0])  //找到距离差最小的摄影无人机
                        {
                            if(teammate=="/jurong")     continue;
                            if(raffles_dis[teammate] - jurong_dis[teammate] < dis_)
                            {
                                dis_ = raffles_dis[teammate] - jurong_dis[teammate] ;
                                transname = teammate;
                            }
                        }
                        team_info[0].erase(std::remove(team_info[0].begin(), team_info[0].end(), transname), team_info[0].end());
                        team_info[1].push_back(transname);
                    }
                }
                transname=team_info[1][1]; 
                if(team_info[1].size()>team_info[0].size())
                {
                    cout << "456" <<endl;
                    while (team_info[1].size()-team_info[0].size()>1)
                    {
                        double dis_= jurong_dis[team_info[1][1]] - raffles_dis[team_info[1][1]];
                        for(auto teammate:team_info[1])  //找到距离差最小的摄影无人机
                        {
                            if(teammate=="/raffles")     continue;
                            if(jurong_dis[teammate] - raffles_dis[teammate] < dis_)
                            {
                                dis_ = jurong_dis[teammate] - raffles_dis[teammate] ;
                                transname = teammate;
                            }
                        }
                        team_info[1].erase(std::remove(team_info[1].begin(), team_info[1].end(), transname), team_info[1].end());
                        team_info[0].push_back(transname);
                    }
                }
            }
               
            for(int i=0;i<2;i++){
                cout<<"team"<<i<<": "<<endl;
                for(int j=0;j<team_info[i].size();j++){
                    cout<<team_info[i][j]<<endl;
                }
                cout<<"team"<<i<<" finished"<<endl;
            }
        }
        
    void Best_first_search() //边界框搜索顺序
    {
        Eigen::Vector3d start_point;
        if(team_info[0].size()>0&&team_info[1].size()>0){
            start_point=position_pair[team_info[0][0]].position;
        }else if(team_info[0].size()>0&&team_info[1].size()==0){
            start_point=position_pair[team_info[0][0]].position;
        }else if(team_info[0].size()==0&&team_info[1].size()>0){
            start_point=position_pair[team_info[1][0]].position;
        }

        while(box_index.size()<box_set.size())
        {
            int index;
            int state;
            double mindis=std::numeric_limits<double>::max();
            for(int i=0;i<box_set.size();i++)
            {
                if(find(box_index.begin(),box_index.end(),i)!=box_index.end()&&box_index.size()>0)
                {
                    continue;
                }
                for(int j=0;j<2;j++)
                {  
                    double dis=(box_set[i].get_global_in_out(j)-start_point).norm();
                    if(dis<mindis)
                    {
                        mindis=dis;
                        state=j;
                        index=i;
                    }
                }
            }
            box_index.push_back(index);
            state_vec.push_back(state);
            start_point=box_set[index].get_global_in_out(1-state);
        }
        cout<<"Path:"<<endl;
        for(int i=0;i<box_set.size();i++)
        {
            cout<<"bbox index:"<<box_index[i];
            cout<<" state:"<<state_vec[i]<<endl;
        }

    }
    void Clip_the_task()  //按边界框体积分配无人机任务范围，保存在output_path[];
    {
        double volum_path = 0;
        int clip_index = -1;
        bool clip_in_boundingbox = false;

        Boundingbox replaced_in;
        Boundingbox replaced_out;
        vector<Boundingbox> BFS_result;
        for(int i=0;i<box_index.size();i++)
        {
            BFS_result.push_back(box_set[box_index[i]]);
            BFS_result[i].edit_state(state_vec[i]);
        }

        output_path.resize(2);

        if(team_info[0].size()==0&&team_info[1].size()>0)
        {
            output_path[1]=BFS_result;
            return;
        }
        else if(team_info[0].size()>0&&team_info[1].size()==0)
        {
            output_path[0]=BFS_result;
            return;
        }
        else if(team_info[0].size()>0&&team_info[1].size()>0)
        {
            double factor=double(team_info[0].size())/double(team_info[0].size()+team_info[1].size());
            cout<<"factor"<<to_string(factor)<<endl;
            for(int j=0;j<BFS_result.size();j++) //按体积分配任务边界框范围
            {
                volum_path +=BFS_result[j].getVolume();
                if (abs(volum_path / volumn_total - factor) <= 0.05)
                {
                    clip_index = j;
                    clip_in_boundingbox = false;
                    break;
                }
                if (volum_path / volumn_total - factor > 0.05)
                {
                
                    clip_index = j;
                    clip_in_boundingbox = true;
                    double volum_more = volum_path - volumn_total * factor;
                    double scaled_param = 1 - volum_more / (BFS_result[j].getVolume());
                    BFS_result[j].generate_start(scaled_param, replaced_in, replaced_out);
                }
            }

            if(clip_in_boundingbox) //对边界框进行了切割
            {
                for (int i = 0; i < BFS_result.size(); i++)
                {
                    if (i < clip_index)
                    {
                        output_path[0].push_back(BFS_result[i]);
                    }
                    else if (i == clip_index)
                    {
                        output_path[0].push_back(replaced_in);
                        output_path[1].push_back(replaced_out);
                    }
                    else
                    {
                        output_path[1].push_back(BFS_result[i]);
                    }
                }
                return;
            }
            else
            {
                for (int i = 0; i < BFS_result.size(); i++)
                {
                    if (i <= clip_index)
                    {
                        output_path[0].push_back(BFS_result[i]);
                    }
                    else
                    {
                        output_path[1].push_back(BFS_result[i]);
                    }
                } 
                return;       

            }
                
        }

    }

    void generate_massage()
    {
        result="";
        double loose_length=6.0;
        result = result + to_string(xmin - loose_length) + ",";
        result = result + to_string(ymin - loose_length) + ",";
        result = result + to_string(zmin - loose_length) + ",";

        result = result + to_string(xmax + loose_length) + ",";
        result = result + to_string(ymax + loose_length) + ",";
        result = result + to_string(zmax + loose_length) + ",";

        result=result+"team"+",0,"+to_string(team_info[0].size())+",";

        for(int i=0;i<team_info[0].size();i++)
        {
            string str=team_info[0][i];
            str.erase(0, 1);
            result=result+str+",";
        }
        result=result+"team"+",1,"+to_string(team_info[1].size())+",";
        for(int i=0;i<team_info[1].size();i++){
            string str=team_info[1][i];
            str.erase(0, 1);
            result=result+str+",";
        }
        result = result + "path_size" + "," + "0" + "," + to_string(output_path[0].size()) + ",";
        result = result + "path_size" + "," + "1" + "," + to_string(output_path[1].size()) + ",";
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < output_path[i].size(); j++)
            {
                result = result + ";";
                result = result + output_path[i][j].generate_string_version();
            }
        }
        finish_massage_generate=true;

    }


};