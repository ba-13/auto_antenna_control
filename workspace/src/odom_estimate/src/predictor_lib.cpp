#include <iostream>
#include "odom_estimate/predictor_lib.hpp"
#include <chrono>
#include <ros/ros.h>

Predictor::Predictor(double time_horizon)
{
    this->time_horizon = time_horizon;
    this->lag = 0;
    this->last_time = 0;
    this->position.setZero();
    this->velocity.setZero();
    this->acceleration.setZero();
}

void Predictor::set_position(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // this is called when subscriber is faster than publisher
    if (msg == nullptr)
        return;
    this->position(0) = msg->pose.position.x;
    this->position(1) = msg->pose.position.y;
    this->position(2) = msg->pose.position.z;
    this->last_time = msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9;
}

void Predictor::set_velocity(const geometry_msgs::Vector3::ConstPtr &msg)
{
    if (msg == nullptr)
        return;
    this->velocity(0) = msg->x;
    this->velocity(1) = msg->y;
    this->velocity(2) = msg->z;
}

void Predictor::set_acceleration(const geometry_msgs::Vector3::ConstPtr &msg)
{
    if (msg == nullptr)
        return;
    this->acceleration(0) = msg->x;
    this->acceleration(1) = msg->y;
    this->acceleration(2) = msg->z;
}

ros::Time Predictor::predict_position()
{
    auto time = ros::Time::now();
    double current_time = time.sec + time.nsec * 1e-9;
    this->lag = current_time - this->last_time;
    double future_time = this->time_horizon + this->lag;
    this->predicted_position = this->position + this->velocity * future_time + 0.5 * this->acceleration * future_time * future_time;
    // convert to r, theta, phi
    this->predicted_position_rtp = cartesian_to_spherical(this->predicted_position);
    return time;
}

Eigen::Vector3f cartesian_to_spherical(Eigen::Vector3f cartesian)
{
    float x = cartesian(0);
    float y = cartesian(1);
    float z = cartesian(2);
    float r_phi_sq = x * x + y * y;
    float r = std::sqrt(r_phi_sq + z * z);
    float theta, phi;

    theta = std::acos(z / r);
    int signY = (y >= 0) ? 1 : -1;
    phi = signY * std::acos(x / std::sqrt(r_phi_sq));

    return Eigen::Vector3f(r, theta, phi);
}