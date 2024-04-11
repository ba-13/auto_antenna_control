#include<ros/ros.h>
#include<std_msgs/Int32.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "motor_pose_sim");
    ros::NodeHandle nh;
    ros::Rate rate(3);

    ros::Publisher motor_pose_pub = nh.advertise<std_msgs::Int32>("motor_pose_sim", 10);

    float freq = 0.2;

    while (ros::ok()) {
        std_msgs::Int32 msg;
        // msg.header.stamp = ros::Time::now();
        msg.data = 300 * sin(freq * ros::Time::now().toSec());
        motor_pose_pub.publish(msg);
        rate.sleep();
    }

  return 0;
}