#include "antenna_control/predictor_lib.hpp"
#include <utility>

int main() {
  MotorPositionPredictor *pred = new MotorPositionPredictor();
  double init_velocity = 3.0;
  int current_steps = 0;
  int timesteps = 1000 / 20;
  int actual_change = -200;
  std::cout << "Enter actual change: ";
  std::cin >> actual_change;
  std::cout << "Required change: " << actual_change << std::endl;
  std::pair<double, double> val =
      pred->get_next_target_change(init_velocity, actual_change, timesteps);
  std::cout << val.first << " " << val.second << "\n";
}