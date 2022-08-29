#include <string> 
#include <iostream> 
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
using namespace std;

static ros::Publisher ndt_map_rgb_pub;
static pcl::PointCloud<pcl::PointXYZRGB> map_rgb;
Eigen::Matrix4d t_localizer_rgb(Eigen::Matrix4d::Identity()); 

void current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& input)
{ 
    tf::Quaternion q;

    float t_x = input->pose.position.x;
    float t_y = input->pose.position.y;
    float t_z = input->pose.position.z;

    tf::quaternionMsgToTF(input->pose.orientation, q);
    Eigen::Quaterniond q_( q.w(), q.x(), q.y(), q.z());
    Eigen::Matrix3d R = q_.toRotationMatrix();  //Quaternion to rotation matrix

    Eigen::Matrix4d t_localizer(Eigen::Matrix4d::Identity());
    t_localizer.block<3, 3>(0, 0) = R;
    t_localizer(0,3) = t_x;
    t_localizer(1,3) = t_y;
    t_localizer(2,3) = t_z;
    t_localizer_rgb = t_localizer;

    //cout << t_localizer << endl << endl;
} 

static void points_rgb_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_scan_ptr2(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*input, *transformed_scan_ptr);
    pcl::transformPointCloud(*transformed_scan_ptr, *transformed_scan_ptr2, t_localizer_rgb);
    map_rgb += *transformed_scan_ptr2;

    sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_rgb_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(map_rgb));
    map_rgb_ptr->header.frame_id = "map";
    pcl::toROSMsg(*map_rgb_ptr, *map_msg_ptr);

    ndt_map_rgb_pub.publish(*map_msg_ptr);
}

static void save_pcd()
{
    pcl::io::savePCDFileBinary("/home/a123/Downloads/RGB_PCD/RgbMap.pcd", map_rgb);
    cout << "-----------save RGB PCD-----------" << endl;
}

int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "my_rgb_mapping");

    ros::NodeHandle nh;
    ros::Subscriber current_pose_sub = nh.subscribe("/current_pose", 10, current_pose_callback);
    ros::Subscriber points_rgb_sub = nh.subscribe("/points_fused", 10, points_rgb_callback);
    ndt_map_rgb_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_rgb_map_2", 10);

    ros::Rate rate(1);
    while (ros::ok())
    {
        rate.sleep();
        ros::spinOnce(); 
    }
    save_pcd();

    return 0; 
}

