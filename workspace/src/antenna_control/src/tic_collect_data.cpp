#include "antenna_control/tic_lib.hpp"
#include <fstream>
#include <utility>
#include <vector>

using namespace data_collection;

void write_positions_to_file(int target) {
  std::ofstream outfile(FILENAME, std::ofstream::app);
  for (auto position : POSITIONS) {
    outfile << position.first << ", " << position.second << "\n";
  }
  outfile << "\n";
  outfile.close();
}

int main() {
  try {
    H = open_handle(nullptr);
  } catch (const std::exception &error) {
    std::cerr << "Handle error: " << error.what() << std::endl;
    return 1;
  }

  TIC_VARS = H.get_variables();
  H.halt_and_set_position(0);
  std::cout << "Resetting current position as 0\n";
  H.energize();

  // keep obtaining current position
  std::thread t_position(thread_get_position);

  std::cout << "Max speed: " << TIC_VARS.get_max_speed() << ".\n";
  std::cout << "Step mode: " << TIC_VARS.get_step_mode() << ".\n";

  std::vector<int> targets = {10, 20,  30,  40,  50,  60,  70,  80,
                              90, 100, 200, 300, 400, 500, 1000};

  H.exit_safe_start();

  FILENAME = "data" + std::to_string(TIC_VARS.get_max_speed()) + ".csv";
  std::ofstream outfile(FILENAME); // clear the content
  outfile.close();

  for (int target : targets) {
    std::cout << "Going to target: " << target << std::endl;
    MUTEX.lock();
    POSITIONS.clear();
    MUTEX.unlock();
    TIME = std::chrono::steady_clock::now();
    WRITE = true;
    // go to target
    H.set_target_position(target);
    std::this_thread::sleep_for(std::chrono::milliseconds(OBSERVATION_TAU));
    // sleep until reached target
    while (std::abs(TIC_VARS.get_current_position() - target) > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(OBSERVATION_TAU));
    }
    std::cout << "Reached target. Going back to Home\n";

    // go back to home
    WRITE = false;
    write_positions_to_file(target); // save to csv
    H.set_target_position(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(OBSERVATION_TAU));
    // sleep until at home
    while (TIC_VARS.get_current_position() > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(OBSERVATION_TAU));
    }
  }

  RUNNING = false;
  t_position.join();

  return 0;
}