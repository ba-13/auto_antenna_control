#include <ros/ros.h>
#include <tic.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "motor_init_node");
  ros::NodeHandle nh;
  double r = 5;
  ros::Rate rate(r);
}