#include <iostream>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>

namespace predictor
{
    class Predictor
    {
    private:
        double time_horizon;
        double lag;
        double last_time;
        Eigen::Vector3f position, velocity, acceleration;

    public:
        Predictor(double time_horizon);
        Eigen::Vector3f predicted_position;
        Eigen::Vector3f predicted_position_rtp;
        void set_position(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void set_velocity(const geometry_msgs::TwistStamped::ConstPtr &msg);
        void set_acceleration(const sensor_msgs::Imu::ConstPtr &msg);
        Eigen::Vector3f predict_position();
        void lag_update();
    };
} // namespace predictor

namespace geometry
{
    Eigen::Vector3f cartesian_to_spherical(Eigen::Vector3f cartesian);
} // namespace geometry
