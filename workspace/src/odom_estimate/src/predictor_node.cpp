#include <iostream>
#include <predictor_lib.hpp>
#include<ros/ros.h>
#include<eigen3/Eigen/Dense>

float rad_to_deg(float rad)
{
    return rad * 180 / M_PI;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "predictor_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1.0);
    predictor::Predictor predictor(1.0);

    ros::Subscriber sub_pose = nh.subscribe("/virtual/pose", 1, &predictor::Predictor::set_position, &predictor);    
    ros::Subscriber sub_vel = nh.subscribe("/virtual/velocity", 1, &predictor::Predictor::set_velocity, &predictor);
    ros::Subscriber sub_acc = nh.subscribe("/virtual/imu", 1, &predictor::Predictor::set_acceleration, &predictor);
    ros::Publisher pub_predicted_position = nh.advertise<geometry_msgs::PoseStamped>("/predicted/pose", 1);
    while(ros::ok())
    {
        predictor.predict_position();
        geometry_msgs::PoseStamped msg;
        msg.pose.position.x = predictor.predicted_position_rtp(0);
        msg.pose.position.y = rad_to_deg(predictor.predicted_position_rtp(1));
        msg.pose.position.z = rad_to_deg(predictor.predicted_position_rtp(2));
        msg.header.stamp = ros::Time::now();
        pub_predicted_position.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}