#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_sim");
    ros::NodeHandle nh;

    ros::Rate loop_rate(1.0);
    Eigen::Vector3f pose(0.0f, 0.0f, 0.0f);
    double radius = 6.0f;
    float angle = 0.0f;
    double offset_x, offset_y, offset_z;
    offset_x = 10.0;
    offset_y = 0.0;
    offset_z = 2.0;
    double omega = 0.6;
    nh.param<double>("x", offset_x, 10.0);
    nh.param<double>("y", offset_y, 0.0);
    nh.param<double>("z", offset_z, 2.0);
    nh.param<double>("radius", radius, 6.0);
    nh.param<double>("omega", omega, 0.6);
    ROS_INFO_STREAM("x " << offset_x << " radius " << radius);

    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/virtual/pose", 1);
    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Vector3>("/virtual/velocity", 1);
    ros::Publisher pub_imu = nh.advertise<geometry_msgs::Vector3>("/virtual/imu", 1);
    while (ros::ok())
    {
        pose.x() = radius * cos(angle) + offset_x;
        pose.y() = radius * sin(angle) + offset_y;
        pose.z() = offset_z;
        angle += (omega / 1.0);

        geometry_msgs::PoseStamped msg;
        msg.pose.position.x = pose.x();
        msg.pose.position.y = pose.y();
        msg.pose.position.z = pose.z();
        msg.header.stamp = ros::Time::now();
        pub_pose.publish(msg);

        // compute velocity
        geometry_msgs::Vector3 vel_msg;
        vel_msg.x = -radius * omega * sin(angle);
        vel_msg.y = radius * omega * cos(angle);
        vel_msg.z = 0.0;
        pub_vel.publish(vel_msg);

        // compute acceleration
        geometry_msgs::Vector3 imu_msg;
        imu_msg.x = -radius * omega * omega * cos(angle);
        imu_msg.y = -radius * omega * omega * sin(angle);
        imu_msg.z = 0.0;
        pub_imu.publish(imu_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}