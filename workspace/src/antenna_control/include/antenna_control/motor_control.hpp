#include "antenna_control/predictor_lib.hpp"
#include "antenna_control/tic_lib.hpp"
#include "messages/MotorPose.h"
#include "messages/AngleStamped.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#define OBS_RATE 50

double target = 0;
int curr_steps = 0;
int curr_vel = 0;
int smooth_vel = 0;

void target_pose_callback(const messages::AngleStamped &msg)
{
  target = msg.angle;
}

void motor_pose_callback(const messages::MotorPose::ConstPtr &msg)
{
  curr_steps = msg->step;
  curr_vel = msg->velocity;
  smooth_vel = msg->smooth_velocity;
}
