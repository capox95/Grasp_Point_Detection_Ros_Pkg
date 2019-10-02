#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include "../include/entropy.h"
#include "../include/binsegmentation.h"
#include "../include/pointpose.h"

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

void builtBroadcasterMsgs(geometry_msgs::PoseStamped &pose_out, std::string parent_frame,
                          std::string child_frame, geometry_msgs::TransformStamped &broadcast)
{
    broadcast.header.frame_id = parent_frame;
    broadcast.child_frame_id = child_frame;

    broadcast.transform.translation.x = pose_out.pose.position.x;
    broadcast.transform.translation.y = pose_out.pose.position.y;
    broadcast.transform.translation.z = pose_out.pose.position.z;

    broadcast.transform.rotation.x = pose_out.pose.orientation.x;
    broadcast.transform.rotation.y = pose_out.pose.orientation.y;
    broadcast.transform.rotation.z = pose_out.pose.orientation.z;
    broadcast.transform.rotation.w = pose_out.pose.orientation.w;

    broadcast.header.stamp = ros::Time::now();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_tf2_listener");
    ros::NodeHandle node;

    // pointcloud IO .pcd --------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *source) == -1)
    {
        PCL_ERROR(" error opening file ");
        return (-1);
    }
    std::cout << "cloud orginal size: " << source->size() << std::endl;
    // -----------------------------------------

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf2_ros::TransformBroadcaster tfBroadcaster;
    geometry_msgs::TransformStamped broadcast;

    ros::Rate rate(10.0);
    while (node.ok())
    {
        geometry_msgs::TransformStamped transform;
        try
        {
            transform = tfBuffer.lookupTransform("world", "rgbd_camera_depth_optical_frame", ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        Eigen::Vector3d position;
        Eigen::Quaterniond rotation;
        grasp_point_callback(source, position, rotation);

        geometry_msgs::Quaternion quat = tf2::toMsg(rotation);
        geometry_msgs::Point pos = tf2::toMsg(position);

        geometry_msgs::PoseStamped pose_in, pose_out;
        pose_in.pose.orientation = quat;
        pose_in.pose.position = pos;
        tf2::doTransform(pose_in, pose_out, transform);

        builtBroadcasterMsgs(pose_out, "world", "grasp_pose", broadcast);
        tfBroadcaster.sendTransform(broadcast);

        rate.sleep();
    }
    return 0;
};