#include <ros/ros.h>
#include <std_msgs/Float32.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "angle_sim");
  ros::NodeHandle nh;
  int r = 5;
  ros::Rate rate(r);

  ros::Publisher motor_pose_pub = nh.advertise<std_msgs::Float32>("/motor_pose_sim_prediction", 5);
  ros::Publisher lagged_command = nh.advertise<std_msgs::Float32>("/motor_pose_sim_curr", 5);

  double omega = 2 * M_PI * 0.2;
  std_msgs::Float32 msg;
  double time_horizon = 1 / r;
  double A = 30; // in degrees

  while (ros::ok())
  {
    auto curr_time = ros::Time::now().toSec();

    msg.data = A * std::sin(omega * (curr_time + time_horizon)); // perfect prediction
    motor_pose_pub.publish(msg);

    msg.data = A * std::sin(omega * curr_time);
    lagged_command.publish(msg);
    rate.sleep();
  }

  return 0;
}