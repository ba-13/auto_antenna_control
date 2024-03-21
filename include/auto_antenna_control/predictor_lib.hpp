#include<iostream>
#include<vector>
#include<eigen3/Eigen/Dense>

namespace predictor {
    class Predictor{
        private:
            double time_horizon;
            Eigen::Vector3f position, velocity, acceleration; 
        public:
            Predictor(double time_horizon);
            Eigen::Vector3f predicted_position;
            void set_position(Eigen::Vector3f position);
            void set_velocity(Eigen::Vector3f velocity);
            void set_acceleration(Eigen::Vector3f acceleration);
            Eigen::Vector3f predict_position();
    };
}