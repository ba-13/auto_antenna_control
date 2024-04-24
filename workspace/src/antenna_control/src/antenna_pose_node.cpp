#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Odometry.h>
#include<messages/SphericalPose.h>
#include<eigen3/Eigen/Dense>

messages::SphericalPose antenna_pos;

void antenna_pose_callback(const nav_msgs::Odometry::ConstPtr& msg){
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    // theta and phi are the angles of the spherical coordinate system
    double theta = atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(q.y()*q.y() + q.z()*q.z()));
    double phi = asin(2*(q.w()*q.y() - q.z()*q.x()));
    antenna_pos.theta = phi*180/M_PI+90;
    antenna_pos.phi = theta*180/M_PI;
}


int main(int argc, char** argv){

    ros::init(argc, argv, "antenna_pose_node");
    ros::NodeHandle nh;
    ros::Publisher antenna_pose_pub = nh.advertise<messages::SphericalPose>("/antenna/pose", 1);
    ros::Subscriber antenna_pose_sub = nh.subscribe("/qualisys/AAT/odom", 10, antenna_pose_callback);
    ros::Rate rate(10);
    
    while(ros::ok()){
        ros::spinOnce();
        antenna_pos.header.stamp = ros::Time::now();
        antenna_pose_pub.publish(antenna_pos);
        rate.sleep();
    }
    return 0;
}