#include <iostream>
#include <predictor_lib.hpp>
#include<chrono>
#include<ros/ros.h>
namespace predictor
{
    Predictor::Predictor(double time_horizon)
    {
        this->time_horizon = time_horizon;
        this->lag = 0;
    }

    void Predictor::set_position(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        this->position(0) = msg->pose.position.x;
        this->position(1) = msg->pose.position.y;
        this->position(2) = msg->pose.position.z;
        this->last_time = msg->header.stamp.sec + msg->header.stamp.nsec * 1e-9;
    }

    void Predictor::set_velocity(const geometry_msgs::TwistStamped::ConstPtr &msg)  {
        this->velocity(0) = msg->twist.linear.x;
        this->velocity(1) = msg->twist.linear.y;
        this->velocity(2) = msg->twist.linear.z;
    }

    void Predictor::set_acceleration(const sensor_msgs::Imu::ConstPtr &msg)
    {
        this->acceleration(0) = msg->linear_acceleration.x;
        this->acceleration(1) = msg->linear_acceleration.y;
        this->acceleration(2) = msg->linear_acceleration.z;        
    }

    Eigen::Vector3f Predictor::predict_position()
    {
        this->predicted_position = this->position + this->velocity * (this->time_horizon+this->lag) + 0.5 * this->acceleration * (this->time_horizon + this->lag) * (this->time_horizon + this->lag);
        // convert to r, theta, phi
        this->predicted_position_rtp = geometry::cartesian_to_spherical(this->predicted_position);
        return this->predicted_position;
    }

    void Predictor::lag_update()
    {
        // update lag to (current_time - last_time) in seconds
        auto time = ros::Time::now();
        double current_time = time.sec + time.nsec * 1e-9;
        this->lag = current_time - this->last_time;
        ROS_WARN("Lag updated to %f", this->lag);
    }
}

namespace geometry
{
    Eigen::Vector3f cartesian_to_spherical(Eigen::Vector3f cartesian)
    {
        // ! FUNCTION IS SUSPECTED TO BE INCORRECT
        // ! PLEASE VERIFY THE FUNCTIONALITY
        float x = cartesian(0);
        float y = cartesian(1);
        float z = cartesian(2);
        float r_phi_sq = x * x + y * y;
        float r = sqrt(r_phi_sq + z * z);
        float theta, phi;

        theta = acos(z / r);
        int signY = (y >= 0) ? 1 : -1;
        phi = signY * acos(x / sqrt(r_phi_sq));

        return Eigen::Vector3f(r, theta, phi);
    }
}