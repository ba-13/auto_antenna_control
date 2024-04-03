#include <iostream>
#include <predictor_lib.hpp>

namespace predictor
{
    Predictor::Predictor(double time_horizon)
    {
        this->time_horizon = time_horizon;
    }

    void Predictor::set_position(Eigen::Vector3f position)
    {
        this->position = position;
    }

    void Predictor::set_velocity(Eigen::Vector3f velocity)
    {
        this->velocity = velocity;
    }

    void Predictor::set_acceleration(Eigen::Vector3f acceleration)
    {
        this->acceleration = acceleration;
    }

    Eigen::Vector3f Predictor::predict_position()
    {
        this->predicted_position = this->position + this->velocity * this->time_horizon + 0.5 * this->acceleration * this->time_horizon * this->time_horizon;
        // convert to r, theta, phi
        this->predicted_position_rtp(0) = this->predicted_position.norm();
        this->predicted_position_rtp(1) = atan2(this->predicted_position(1), this->predicted_position(0));
        this->predicted_position_rtp(2) = acos(this->predicted_position(2) / this->predicted_position.norm());
        return this->predicted_position;
    }
}

namespace geometry
{
    Eigen::Vector3f cartesian_to_spherical(Eigen::Vector3f cartesian)
    {
        float x = cartesian(0);
        float y = cartesian(1);
        float z = cartesian(2);
        float r_phi_sq = x * x + y * y;
        float r = sqrt(r_phi_sq + z * z);
        float theta, phi;

        theta = acos(z / r);
        int signY = (y >= 0) ? 1 : -1;
        phi = signY * acos(x / sqrt(r_phi_sq));

        return Eigen::Vector3f(r, theta, phi);
    }
}