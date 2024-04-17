#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "angle_sim");
  ros::NodeHandle nh;
  int r;
  double factorA, factorB;

  nh.param<double>("A", factorA, 8.88888888);
  nh.param<double>("B", factorB, 0);
  nh.param<int>("prediction_pose_rate", r, 3);
  ros::Rate rate(r);

  ros::Publisher motor_pose_pub = nh.advertise<std_msgs::Float32>("/motor_pose_sim_prediction", 5);
  ros::Publisher lagged_command = nh.advertise<std_msgs::Int32>("/motor_pose_sim_curr", 5);

  double omega = 2 * M_PI * 0.2;
  std_msgs::Float32 msgF;
  double time_horizon = 1 / r;
  double A = 30; // in degrees

  while (ros::ok())
  {
    auto curr_time = ros::Time::now().toSec();

    msgF.data = A * std::sin(omega * (curr_time + time_horizon)); // perfect prediction
    ROS_WARN_STREAM("target from sim: " << int(msgF.data * factorA + factorB));
    motor_pose_pub.publish(msgF);

    double data = A * std::sin(omega * curr_time);
    lagged_command.publish(msgF);
    rate.sleep();
  }

  return 0;
}