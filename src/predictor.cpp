#include<iostream>
#include<auto_antenna_control/predictor_lib.hpp>

namespace predictor{
    Predictor::Predictor(double time_horizon){
        this->time_horizon = time_horizon;
    }

    void Predictor::set_position(Eigen::Vector3f position){
        this->position = position;
    }

    void Predictor::set_velocity(Eigen::Vector3f velocity){
        this->velocity = velocity;
    }

    void Predictor::set_acceleration(Eigen::Vector3f acceleration){
        this->acceleration = acceleration;
    }

    Eigen::Vector3f Predictor::predict_position(){
        this->predicted_position = this->position + this->velocity * this->time_horizon + 0.5 * this->acceleration * this->time_horizon * this->time_horizon;
        return this->predicted_position;
    }
}