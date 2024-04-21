#include "antenna_control/motor_control.hpp"
#include <std_msgs/Int32.h>

int main(int argc, char **argv)
{
  int r;
  std::string motor_handle;
  double factorA, degree_at_step0, max_degrees, factorB;

  ros::init(argc, argv, "motor_control");
  ros::NodeHandle nh;

  nh.param<std::string>("handle", motor_handle, "none");
  ROS_WARN_STREAM(ros::this_node::getName() << " - picked handle: " << motor_handle);
  nh.param<int>("prediction_pose_rate", r, 3);

  nh.param<double>("A", factorA, 8.88888888);
  nh.param<double>("degree_at_step0", degree_at_step0, 10);
  nh.param<double>("max_degrees", max_degrees, 85);
  factorB = -factorA * degree_at_step0; // this much steps is how much can the motor go in negative degrees

  ros::Rate rate(r);
  ros::Subscriber motor_pose_sub = nh.subscribe("/target/predicted/pose/angle", r, target_pose_callback);
  ros::Subscriber motor_info_sub = nh.subscribe("/motor/pose", r, motor_pose_callback);
  ros::Publisher motor_commands = nh.advertise<std_msgs::Float32>("/motor/pose/angle", 1);
  tic::handle H(nullptr);

  if (motor_handle != "none")
    H = open_handle(motor_handle.c_str());
  else
    H = open_handle(nullptr);

  H.halt_and_set_position(0);
  ROS_WARN_STREAM(ros::this_node::getName() << " Resetting current position as 0\n");
  H.energize();

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

    if (target > max_degrees)
    {
      ROS_WARN_STREAM(ros::this_node::getName() << " Capping " << target << " angle to maximum" << max_degrees);
      target = max_degrees;
    }

    int target_steps = predictor->theta_to_steps(target);
    auto command = predictor->get_next_target_change(smooth_vel, target_steps - curr_steps, 1000 / (20 * r));
    // std::pair<double, double> command = {0, target_steps - curr_steps}; // not using predictor

    std_msgs::Float32 current_pose;
    H.set_target_position(curr_steps + command.second);
    current_pose.data = predictor->steps_to_theta(curr_steps);
    motor_commands.publish(current_pose);

    ROS_INFO("%s target_rec: %f %d; %lf == %d; curr: %d", ros::this_node::getName().c_str(), target, target_steps, command.first, target_steps - curr_steps, curr_steps);
    rate.sleep();
  }

  return 0;
}