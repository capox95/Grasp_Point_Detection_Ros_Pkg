#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Publisher pub;
ros::Subscriber sub, sub2;
geometry_msgs::TransformStamped transform;
geometry_msgs::PoseStamped pose;

void transform_cb(geometry_msgs::TransformStamped tf)
{
    transform = tf;
    ROS_INFO("transform received");
}

void grasp_pose_cb(geometry_msgs::PoseStamped p)
{
    pose = p;
    ROS_INFO("pose received");
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
    // Initialize ROS
    ros::init(argc, argv, "grasp_pose_world_frame");
    ros::NodeHandle node;
    ros::Rate rate(10.0);

    tf2_ros::TransformBroadcaster tfBroadcaster;
    geometry_msgs::TransformStamped broadcast;

    sub = node.subscribe("/transform", 1, transform_cb);
    sub2 = node.subscribe("/grasp_pose_camera_frame", 1, grasp_pose_cb);

    geometry_msgs::PoseStamped pose_out;
    tf2::doTransform(pose, pose_out, transform);

    builtBroadcasterMsgs(pose_out, "world", "grasp_pose_world", broadcast);
    tfBroadcaster.sendTransform(broadcast);

    pub = node.advertise<geometry_msgs::PoseStamped>("/grasp_pose_world_frame", 1);
    pub.publish(pose_out);
    ROS_INFO("published!");

    ros::spin();
}