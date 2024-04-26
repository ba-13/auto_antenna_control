#include "antenna_control/predictor_lib.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include "messages/SphericalPose.h"

Eigen::Vector3d rpy;

void fc_imu_callback(const sensor_msgs::ImuConstPtr &msg)
{
    geometry_msgs::Quaternion q = msg->orientation;
    Eigen::Quaterniond qd(q.w, q.x, q.y, q.z);
    rpy = quaternion_to_rpy(qd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "antenna_fc_pose");
    ros::NodeHandle nh;
    ros::Rate r(2);

    ros::Subscriber fc_imu_sub = nh.subscribe("/sitl/mavros/imu/data", 2, fc_imu_callback);
    ros::Publisher fc_imu_pub = nh.advertise<messages::SphericalPose>("/sitl/mavros/imu/rpy", 2);

    while (ros::ok())
    {
        ros::spinOnce();
        messages::SphericalPose msg;
        msg.header.stamp = ros::Time::now();
        msg.radius = 0;
        msg.phi = rpy.z();
        msg.radius = rpy.x();
    }

    return 0;
}