#include<iostream>
#include<ros/ros.h>
#include<eigen3/Eigen/Dense>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/TwistStamped.h>
#include<sensor_msgs/Imu.h>
#include<cmath>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim_drone_node");
    ros::NodeHandle nh;

    ros::Rate loop_rate(1.0);
    Eigen::Vector3f pose(0.0f, 0.0f, 0.0f);
    float radius = 3.0f;
    float angle = 0.0f;
    double offset_x, offset_y, offset_z;
    offset_x = 10.0;
    offset_y = 0.0;
    offset_z = 4.0;
    double omega = 0.6;
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/virtual/pose", 1);
    ros::Publisher pub_vel = nh.advertise<geometry_msgs::TwistStamped>("/virtual/velocity", 1);
    ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("/virtual/imu", 1);
    while(ros::ok())
    {
        // ROS_INFO("Hello, I am a simulated drone.");
        pose.x() = radius * cos(angle) + offset_x;
        pose.y() = radius * sin(angle) + offset_y;
        pose.z() = offset_z;
        angle += (omega/1.0);
        geometry_msgs::PoseStamped msg;
        msg.pose.position.x = pose.x();
        msg.pose.position.y = pose.y();
        msg.pose.position.z = pose.z();
        msg.header.stamp = ros::Time::now();
        pub_pose.publish(msg);

        geometry_msgs::TwistStamped vel_msg;
        vel_msg.twist.linear.x = -radius * omega * sin(angle);  
        vel_msg.twist.linear.y = radius * omega * cos(angle);
        vel_msg.twist.linear.z = 0.0;
        vel_msg.header.stamp = ros::Time::now();
        pub_vel.publish(vel_msg);

        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        // compute acceleration
        imu_msg.linear_acceleration.x = -radius * omega * omega * cos(angle);
        imu_msg.linear_acceleration.y = -radius * omega * omega * sin(angle);
        imu_msg.linear_acceleration.z = 0.0;
        pub_imu.publish(imu_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}