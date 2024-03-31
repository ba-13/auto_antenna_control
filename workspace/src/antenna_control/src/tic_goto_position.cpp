#include "tic_lib.hpp"
#include <thread>
#include <vector>
#include <utility>
#include <mutex>
#include <fstream>

tic::variables TIC_VARS;
tic::handle H(nullptr);
const int OBSERVATION_TAU = 5;
bool RUNNING = true;
bool WRITE = false;
std::chrono::_V2::steady_clock::time_point TIME;
std::vector<std::pair<int64_t, int>> POSITIONS;
std::mutex MUTEX;
std::string FILENAME;

void thread_get_position()
{
    while (RUNNING)
    {
        auto start_time = std::chrono::steady_clock::now();

        // Sample TIC_VARS
        try
        {
            TIC_VARS = H.get_variables();
        }
        catch (const std::exception &error)
        {
            std::cerr << "Handle error: " << error.what() << std::endl;
            continue;
        }

        if (WRITE)
        {
            // have to store
            auto since = std::chrono::duration_cast<std::chrono::milliseconds>(start_time - TIME);

            MUTEX.lock();
            // most prolly push_back when increasing size would
            // lead to sampling time deviation
            POSITIONS.push_back({since.count(), TIC_VARS.get_current_position()});
            MUTEX.unlock();
        }

        // sleep for remaining time
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        auto remaining_time = std::chrono::milliseconds(OBSERVATION_TAU) - elapsed_time;

        if (remaining_time.count() > 0)
        {
            std::this_thread::sleep_for(remaining_time);
        }
    }
}

void write_positions_to_file(int target)
{
    std::ofstream outfile(FILENAME, std::ofstream::app);
    for (auto position : POSITIONS)
    {
        outfile << position.first << ", " << position.second << "\n";
    }
    outfile << "\n";
    outfile.close();
}

int main()
{
    try
    {
        H = open_handle(nullptr);
    }
    catch (const std::exception &error)
    {
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

    std::vector<int> targets = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 200, 300, 400, 500, 1000};

    H.exit_safe_start();

    FILENAME = "data" + std::to_string(TIC_VARS.get_max_speed()) + ".csv";
    std::ofstream outfile(FILENAME); // clear the content
    outfile.close();

    for (int target : targets)
    {
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
        while (std::abs(TIC_VARS.get_current_position() - target) > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(OBSERVATION_TAU));
        }
        std::cout << "Reached target. Going back to Home\n";

        // go back to home
        WRITE = false;
        write_positions_to_file(target); // save to csv
        H.set_target_position(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(OBSERVATION_TAU));
        // sleep until at home
        while (TIC_VARS.get_current_position() > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(OBSERVATION_TAU));
        }
    }

    RUNNING = false;
    t_position.join();

    return 0;
}