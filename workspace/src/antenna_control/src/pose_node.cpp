#include "antenna_control/tic_lib.hpp"
#include <chrono>
#include <messages/MotorPose.h>
#include <ros/ros.h>
#include <tic.hpp>

int main(int argc, char **argv) {
  int RATE = 50;
  ros::init(argc, argv, "motor_pose_node");
  ros::NodeHandle nh;
  std::string motor_name;
  std::string motor_handle;
  nh.param<std::string>("motor", motor_name, "phi");
  nh.param<std::string>("motor/" + motor_name, motor_handle, "");
  ros::Rate loop_rate(RATE);
  nh.param<int>("motor/rate", RATE, 50);

  double m1, m2;
  nh.param<double>("motor/" + motor_name + "/m1", m1, 0.0159136635);
  nh.param<double>("motor/" + motor_name + "/m2", m2, -0.0172123864);

  tic::variables TIC_VARS;
  tic::handle H(nullptr);

  if (motor_handle != "")
    H = open_handle(motor_handle.c_str());
  else
    H = open_handle(nullptr);

  ros::Publisher pub_motor_pose =
      nh.advertise<messages::MotorPose>("/motor/" + motor_name + "/pose", 10);

  int prev = 0, pprev = 0; // assuming no overflow
  float prev_vel = 0;
  int over = 10;

  while (ros::ok()) {
    try {
      TIC_VARS = H.get_variables();
    } catch (const std::exception &error) {
      ROS_ERROR_STREAM("Handle error: " << error.what());
      if (over <= 0)
        return 1;
      loop_rate.sleep();
      over--;
      continue;
    }

    messages::MotorPose pose;
    int p = TIC_VARS.get_current_position();
    int v = TIC_VARS.get_current_velocity();
    pose.step = p;
    pose.velocity = (v / 10000) / RATE; // pulses per tau
    pose.smooth_velocity = (p - pprev) / 2.0;
    if (pose.velocity > 0) {
      pose.smooth_velocity += m1 * 1000 / RATE;
    } else if (pose.velocity < 0) {
      pose.smooth_velocity -= m2 * 1000 / RATE;
    }

    pose.header.stamp = ros::Time::now();
    pub_motor_pose.publish(pose);

    // end with
    prev_vel = v;
    pprev = prev;
    prev = p;
    loop_rate.sleep();
  }
  return 0;
}