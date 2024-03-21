#include<iostream>
#include<auto_antenna_control/predictor_lib.hpp>
#include<unistd.h>

void give_random_updates(predictor::Predictor* predictor){
    predictor->set_position(predictor->predicted_position);
    predictor->set_velocity(Eigen::Vector3f::Random());
    predictor->set_acceleration(Eigen::Vector3f::Random());
}

int main(){
    predictor::Predictor* predictor = new predictor::Predictor(1.0);
    predictor->set_position(Eigen::Vector3f::Random());
    predictor->set_velocity(Eigen::Vector3f::Random());
    predictor->set_acceleration(Eigen::Vector3f::Random());

    while(1){
        predictor->predict_position();
        std::cout << "Predicted position: " << predictor->predicted_position.transpose() << std::endl;
        std::cout << "Predicted position in rtp: " << predictor->predicted_position_rtp.transpose() << std::endl;
        give_random_updates(predictor);
        // sleep for 0.5 seconds
        usleep(500000);
    }

}