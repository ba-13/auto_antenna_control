#include <ros/ros.h>
#include <std_msgs/Int32.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_pose_sim");
  ros::NodeHandle nh;
  float r = 5;
  ros::Rate rate(r);

  ros::Publisher motor_pose_pub = nh.advertise<std_msgs::Int32>("/motor_pose_sim", 5);
  ros::Publisher lagged_command = nh.advertise<std_msgs::Int32>("/motor_pose_sim_lagged", 5);
  float freq = 0.2;

  while (ros::ok())
  {
    std_msgs::Int32 msg;
    double A = 300;
    // msg.header.stamp = ros::Time::now();
    msg.data = A * sin(2 * M_PI * freq * ros::Time::now().toSec());
    motor_pose_pub.publish(msg);
    // msg.header.stamp = ros::Time::now();
    msg.data = A * sin(2 * M_PI * freq * (ros::Time::now().toSec()-(1/r)));
    lagged_command.publish(msg);
    rate.sleep();
  }

  return 0;
}