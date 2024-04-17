#include <ros/ros.h>
#include "antenna_control/tic_lib.hpp"

// void orientation_callback(const geometry)

int main(int argc, char **argv)
{
  ros::init(argc, argv, "initialization");
  ros::NodeHandle nh;
  int R;
  std::string motor_handle;
  nh.param<int>("prediction_pose_rate", R, 3);
  nh.param<std::string>("handle", motor_handle, "");
  ROS_WARN_STREAM(ros::this_node::getName() << " - picked handle: " << motor_handle);

  // ros::Subscriber orientation_sub = nh.subscribe("/antenna/orientation", 2, orientation_callback);

  ros::Rate rate(R);
  tic::handle H(nullptr);
  if (motor_handle != "")
    H = open_handle(motor_handle.c_str());
  else
    H = open_handle(nullptr);

  H.energize();
  tic::variables vars = H.get_variables();
  if (!vars.get_energized())
  {
    ROS_ERROR_STREAM("Motor not energized, exiting " << ros::this_node::getName());
    return 1;
  }

  H.set_target_velocity(-200 * 10000);
  // wait till there's no change in the position
  while (H.get_variables().get_current_velocity() != 0)
  {
    ROS_INFO("Sleeping until reaching negative end");
    rate.sleep();
  }

  H.halt_and_set_position(0); // set this position as 0

  return 0;
}