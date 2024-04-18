#include <ros/ros.h>
#include "antenna_control/tic_lib.hpp"
#include <geometry_msgs/Quaternion.h>
#include <eigen3/Eigen/Dense>
// void orientation_callback(const geometry)

float offset_azimuth = 0.0;

void antenna_orientation_callback(const geometry_msgs::Quaternion::ConstPtr &msg)
{
  Eigen::Quaterniond q(msg->w, msg->x, msg->y, msg->z);
  Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
  // get yaw
  offset_azimuth = euler[2];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "initialization");
  ros::NodeHandle nh;
  double R;
  std::string motor_handle;
  nh.param<double>("prediction_pose_rate", R, 3);
  nh.param<std::string>("handle", motor_handle, "none");
  ROS_WARN_STREAM(ros::this_node::getName() << " - picked handle: " << motor_handle);

  // ros::Subscriber orientation_sub = nh.subscribe("/antenna/orientation", 2, orientation_callback);

  ros::Rate rate(R);
  tic::handle H(nullptr);
  if (motor_handle == "none")
    return 1;
  else
    H = open_handle(motor_handle.c_str());

  H.energize();
  tic::variables vars = H.get_variables();
  if (!vars.get_energized())
  {
    ROS_ERROR_STREAM("Motor not energized, exiting " << ros::this_node::getName());
    return 1;
  }

  H.set_target_velocity(-200 * 10000);
  rate.sleep();
  rate.sleep();
  rate.sleep();
  // wait till there's no change in the position
  while (H.get_variables().get_current_velocity() != 0)
  {
    ROS_INFO("Sleeping until reaching negative end");
    rate.sleep();
  }

  H.halt_and_set_position(0); // set this position as 0
  // ros::Rate r(10);
  // int i = 0;
  // while (ros::ok())
  // {
  //   ros::spinOnce();
  //   r.sleep();
  //   i++;
  //   if (i == 50)
  //     break;
  // }
  // nh.setParam("offset_azimuth", offset_azimuth);
  return 0;
}
