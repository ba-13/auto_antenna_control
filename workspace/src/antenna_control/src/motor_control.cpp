#include "antenna_control/predictor_lib.hpp"
#include "antenna_control/tic_lib.hpp"
#include "messages/MotorPose.h"
#include <ros/ros.h>
#include <std_msgs/Int32.h>
int target = 0;
int curr_vel = 0;
int curr_pos = 0;
int smooth_vel = 0;

void motor_pose_callback(const std_msgs::Int32::ConstPtr &msg) {
  target = msg->data;
}

void motor_info_callback(const messages::MotorPose::ConstPtr &msg) {
  curr_vel = msg->velocity;
  curr_pos = msg->step;
  smooth_vel = msg->smooth_velocity;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "motor_control");
  ros::NodeHandle nh;
  double r = 5;
  ros::Rate rate(r);

  ros::Subscriber motor_pose_sub =
      nh.subscribe("/motor_pose_sim", 5, motor_pose_callback);
  ros::Subscriber motor_info_sub =
      nh.subscribe("/motor/phi/pose", 5, motor_info_callback);
  ros::Publisher motor_commands =
      nh.advertise<std_msgs::Int32>("/motor/phi/command", 5);
  tic::handle H = open_handle(nullptr);

  H.halt_and_set_position(0);
  std::cout << "Resetting current position as 0\n";
  H.energize();

  MotorPositionPredictor *predictor = new MotorPositionPredictor();
  while (ros::ok()) {
    ros::spinOnce();
    auto command = predictor->get_next_target_change(smooth_vel,
                                                     target - curr_pos, 50 / r);

    // std::pair<double, double> command = {0, target - curr_pos};
    H.set_target_position(curr_pos + command.second);
    std_msgs::Int32 command_msg;
    command_msg.data = curr_pos + command.second;
    ROS_ERROR("Command: %lf %lf", command.first, curr_pos + command.second);
    motor_commands.publish(command_msg);
    rate.sleep();
  }

  return 0;
}