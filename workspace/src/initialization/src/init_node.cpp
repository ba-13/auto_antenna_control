#include <ros/ros.h>
#include "antenna_control/tic_lib.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_init_node");
  ros::NodeHandle nh;
  int R;
  std::string motor_handle;
  nh.param<int>("prediction_pose_rate", R, 3);
  nh.param<std::string>("handle", motor_handle, "");
  ROS_WARN_STREAM(ros::this_node::getName() << " - picked handle: " << motor_handle);
  tic::handle H(nullptr);
  if (motor_handle != "")
    H = open_handle(motor_handle.c_str());
  else
    H = open_handle(nullptr);
}