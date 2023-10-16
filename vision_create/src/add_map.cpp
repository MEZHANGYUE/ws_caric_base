#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

class PointCloudAccumulator
{
public:
    PointCloudAccumulator() : accumulated_cloud(new pcl::PointCloud<pcl::PointXYZ>)
    {
        pointcloud_subscriber = nh.subscribe("/gcs/nbr_kf_cloud", 1, &PointCloudAccumulator::pointcloudCallback, this);
        pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/accumulated_pointcloud", 1);
    }

    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZ> input_cloud;
        pcl::fromROSMsg(*cloud_msg, input_cloud);

        // Perform point cloud accumulation here
        *accumulated_cloud += input_cloud;
        accumulated = *accumulated_cloud;

        // Publish the accumulated point cloud
        sensor_msgs::PointCloud2 accumulated_cloud_msg;
        pcl::toROSMsg(*accumulated_cloud, accumulated_cloud_msg);
        accumulated_cloud_msg.header = cloud_msg->header;
        pointcloud_publisher.publish(accumulated_cloud_msg);

        std::string savePath = "/home/lab318/ws_caric/src/pc_subsrciber/models"; // 修改为你希望保存的文件夹路径
        std::string fileName = "test_pcd0"; // 修改为你希望的文件名
        pcl::io::savePCDFileASCII(savePath + "/" +fileName + ".pcd", accumulated);
        ROS_INFO("Point cloud saved to %s", (savePath + "/" + fileName + ".pcd").c_str());
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber pointcloud_subscriber;
    ros::Publisher pointcloud_publisher;
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud;
    pcl::PointCloud<pcl::PointXYZ> accumulated;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_accumulator");
    PointCloudAccumulator accumulator;
    ros::spin();
    return 0;
}