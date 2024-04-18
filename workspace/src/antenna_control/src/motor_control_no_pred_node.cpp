#include "antenna_control/motor_control.hpp"
#include <std_msgs/Int32.h>

int main(int argc, char **argv)
{
  int r;
  std::string motor_handle;
  double factorA, factorB;

  ros::init(argc, argv, "motor_control");
  ros::NodeHandle nh;

  nh.param<std::string>("handle", motor_handle, "");
  nh.param<int>("prediction_pose_rate", r, 3);

  nh.param<double>("A", factorA, 8.88888888);
  nh.param<double>("B", factorB, 0);
  ROS_WARN_STREAM(ros::this_node::getName() << " - picked handle: " << motor_handle);

  ros::Rate rate(r);
  ros::Subscriber motor_pose_sub = nh.subscribe("/target/predicted/pose", r, target_pose_callback);
  ros::Subscriber motor_info_sub = nh.subscribe("/motor/pose", r, motor_pose_callback);
  ros::Publisher motor_commands = nh.advertise<std_msgs::Float32>("/motor/command", 5);
  tic::handle H(nullptr);

  if (motor_handle != "")
    H = open_handle(motor_handle.c_str());
  else
    H = open_handle(nullptr);

  // H.halt_and_set_position(0);
  // ROS_WARN_STREAM("Resetting current position as 0\n");
  // H.energize();
  tic::variables vars = H.get_variables();
  if (!vars.get_energized())
  {
    ROS_ERROR_STREAM("Motor not energized, exiting " << ros::this_node::getName());
    return 1;
  }
  if (vars.get_position_uncertain())
  {
    ROS_ERROR_STREAM("Motor position uncertain, exiting " << ros::this_node::getName());
    return 1;
  }

  double timesteps = OBS_RATE / r;

  MotorPositionPredictor *predictor = new MotorPositionPredictor(factorA, factorB);
  while (ros::ok())
  {
    ros::spinOnce();

    int target_steps = predictor->translate_to_steps(target);
    // auto command = predictor->get_next_target_change(smooth_vel, target_steps - curr_steps, 1000 / (20 * r));
    ROS_INFO_STREAM("curr steps: " << curr_steps);
    std::pair<double, double> command = {0, target_steps - curr_steps}; // not using predictor

    std_msgs::Float32 command_msg;
    command_msg.data = (curr_steps + command.second - factorB); // publishing theta
    H.set_target_position(command_msg.data);
    command_msg.data = command_msg.data / factorA;
    ROS_INFO("Command: %d %lf %d %f", target_steps, command.first, target_steps - curr_steps, command_msg.data);
    motor_commands.publish(command_msg);
    rate.sleep();
  }

  return 0;
}