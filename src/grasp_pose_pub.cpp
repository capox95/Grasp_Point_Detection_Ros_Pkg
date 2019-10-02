#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "../include/entropy.h"
#include "../include/binsegmentation.h"
#include "../include/pointpose.h"

ros::Publisher pub;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr source(new pcl::PointCloud<pcl::PointXYZRGB>); // to be remove!

void grasp_point_callback(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source, Eigen::Vector3d &pos, Eigen::Quaterniond &rot)
{

    //BIN SEGMENTATION -----------------------------------------------------------------------
    BinSegmentation bin;
    bin.setInputCloud(source);
    bin.setNumberLines(4);
    bin.setScaleFactorHullBorders(0.2);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_grasp(new pcl::PointCloud<pcl::PointXYZRGB>);
    bin.compute(cloud_grasp);

    pcl::ModelCoefficients::Ptr plane = bin.getPlaneGroundPoints();

    // ENTROPY FILTER -----------------------------------------------------------------------
    //
    EntropyFilter ef;
    ef.setInputCloud(cloud_grasp);
    ef.setDownsampleLeafSize(0.005);
    ef.setEntropyThreshold(0.7);
    ef.setKLocalSearch(500);        // Nearest Neighbour Local Search
    ef.setCurvatureThreshold(0.01); //Curvature Threshold for the computation of Entropy
    ef.setDepthThreshold(0.03);
    ef.setAngleThresholdForConvexity(5);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result(new pcl::PointCloud<pcl::PointXYZ>);
    ef.compute(cloud_result);

    // GRASP POINT --------------------------------------------------------------------------
    PointPose pp;
    pp.setSourceCloud(source);
    pp.setRefPlane(plane);
    pp.setInputCloud(cloud_result);
    pp.computeGraspPoint();

    Eigen::Vector3f p = pp.getTranslation();
    Eigen::Quaternionf r = pp.getRotation();

    pos = p.cast<double>();
    rot = r.cast<double>();
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    std::cout << "cloud_xyz size: " << cloud->size() << std::endl;

    Eigen::Vector3d position;
    Eigen::Quaterniond rotation;
    grasp_point_callback(source, position, rotation); //should be cloud instead of source, only for testing

    geometry_msgs::Quaternion quat = tf2::toMsg(rotation);
    geometry_msgs::Point pos = tf2::toMsg(position);

    geometry_msgs::PoseStamped pose_grasp;
    pose_grasp.pose.orientation = quat;
    pose_grasp.pose.position = pos;

    pub.publish(pose_grasp);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pcl_node");
    ros::NodeHandle nh;

    // pointcloud IO .pcd --------------------                          // to
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *source) == -1) // be
    {                                                                   // remove
        PCL_ERROR(" error opening file ");                              // later
        return (-1);                                                    //  !
    }                                                                   //  !
    std::cout << "cloud orginal size: " << source->size() << std::endl; //  !
    // -----------------------------------------

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/rgbd_camera/rgb/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<geometry_msgs::PoseStamped>("/grasp_pose_camera_frame", 1);

    // Spin
    ros::spin();
}