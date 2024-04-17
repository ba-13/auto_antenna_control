#include <cmath>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>

class Predictor
{
private:
    double time_horizon;
    double lag;
    double last_time;
    Eigen::Vector3f position, velocity, acceleration;

public:
    // time horizon is the time after current time the pose is for
    Predictor(double time_horizon);
    Eigen::Vector3f predicted_position;
    Eigen::Vector3f predicted_position_rtp;
    void set_position(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void set_velocity(const geometry_msgs::Vector3::ConstPtr &msg);
    void set_acceleration(const geometry_msgs::Vector3::ConstPtr &msg);
    ros::Time predict_position();
};

Eigen::Vector3f cartesian_to_spherical(Eigen::Vector3f cartesian);
