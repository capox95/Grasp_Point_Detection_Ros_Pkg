#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

ros::Publisher pub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf2_transform_pub");
    ros::NodeHandle node;
    ros::Rate rate(10.0);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    pub = node.advertise<geometry_msgs::TransformStamped>("/transform", 1);

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

        pub.publish(transform);

        rate.sleep();
    }
    return 0;
};