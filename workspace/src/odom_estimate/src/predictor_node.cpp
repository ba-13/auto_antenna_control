#include <iostream>
#include <predictor_lib.hpp>
#include <unistd.h>
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

void give_random_updates(predictor::Predictor *predictor)
{
    predictor->set_position(predictor->predicted_position);
    predictor->set_velocity(Eigen::Vector3f::Random());
    predictor->set_acceleration(Eigen::Vector3f::Random());
}

float rad_to_deg(float rad)
{
    return rad * 180 / M_PI;
}

int main()
{
    Eigen::Vector3f position({-1, -1, 1});
    Eigen::Vector3f spherical = geometry::cartesian_to_spherical(position);
    std::cout << position(0) << " " << position(1) << " " << position(2) << std::endl
              << spherical(0) << " " << rad_to_deg(spherical(1)) << " " << rad_to_deg(spherical(2)) << std::endl;
    std::vector<float> X({0}), Y({0}), Z({0});
    X.push_back(position(0)), Y.push_back(position(1)), Z.push_back(position(2));
    plt::scatter(X, Y, Z);
    plt::show();
    return 0;
}