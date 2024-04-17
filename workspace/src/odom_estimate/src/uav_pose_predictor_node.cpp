#include <ros/ros.h>
#include "odom_estimate/predictor_lib.hpp"
#include "messages/SphericalPose.h"

float rad_to_deg(float rad) { return rad * 180 / M_PI; }

int main(int argc, char **argv)
{
  int rate, times;
  ros::init(argc, argv, "uav_pose_predictor");
  ros::NodeHandle nh;
  nh.param<int>("incoming_pose_rate", rate, 1);              // rate of incoming pose
  nh.param<int>("prediction_interpolation_times", times, 1); // rate of publishing pose
  ROS_WARN_STREAM("rate: " << rate << " times: " << times);

  ros::Rate loop_rate(rate * times);
  Predictor predictor(1.0 / (rate * times)); // time horizon is 1/R seconds

  // TODO these topics take the data directly from the drone
  //! we need to offset this pose so antenna is at origin before finding
  //! rtp and publishing
  ros::Subscriber sub_pose = nh.subscribe("/current/pose", 1, &Predictor::set_position, &predictor);
  ros::Subscriber sub_vel = nh.subscribe("/current/velocity", 1, &Predictor::set_velocity, &predictor);
  ros::Subscriber sub_acc = nh.subscribe("/current/acceleration", 1, &Predictor::set_acceleration, &predictor);
  ros::Publisher pub_predicted_position = nh.advertise<geometry_msgs::PoseStamped>("/predicted/pose", 1);

  int counter = -1;
  messages::SphericalPose msg;
  while (ros::ok())
  {
    counter++;
    if (counter % times == 0) // call every rate Hz
    {
      ros::spinOnce(); // callbacks and sets last seen time
      counter = 0;
    }
    msg.header.stamp = predictor.predict_position();
    msg.radius = predictor.predicted_position_rtp(0);
    msg.theta = rad_to_deg(predictor.predicted_position_rtp(1)); // sent in degrees
    msg.phi = rad_to_deg(predictor.predicted_position_rtp(2));
    pub_predicted_position.publish(msg);
    loop_rate.sleep(); // call every rate*times Hz
  }
  return 0;
}