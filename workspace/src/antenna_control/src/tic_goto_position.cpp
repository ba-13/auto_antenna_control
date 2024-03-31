#include "tic_lib.hpp"
#include <thread>
#include <vector>
#include <utility>
#include <mutex>
#include <fstream>

tic::variables TIC_VARS;
tic::handle H(nullptr);
const int OBSERVATION_TAU = 20;
bool RUNNING = true;
bool WRITE = false;
std::chrono::_V2::steady_clock::time_point TIME;
std::vector<std::pair<int64_t, int>> POSITIONS;
std::mutex MUTEX;

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
            return;
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
    std::ofstream outfile("data.csv", std::ofstream::app);
    for (auto position : POSITIONS)
    {
        outfile << position.first << ", " << position.second << "\n";
    }
    outfile << "\n";
    outfile.close();
}

int main()
{
    tic::handle handle;
    try
    {
        handle = open_handle(nullptr);
    }
    catch (const std::exception &error)
    {
        std::cerr << "Handle error: " << error.what() << std::endl;
        return 1;
    }

    tic::variables vars = handle.get_variables();
    handle.halt_and_set_position(0);
    handle.energize();

    // keep obtaining current position
    std::thread t_position(thread_get_position);

    std::cout << "Max speed is:" << vars.get_max_speed() << ".\n";

    std::vector<int> targets = {1, 3, 10, 30, 100, 300, 1000};

    handle.exit_safe_start();

    std::ofstream outfile("data.csv"); // clear the content
    outfile.close();

    for (int target : targets)
    {
        while (true)
        {
            MUTEX.lock();
            POSITIONS.clear();
            MUTEX.unlock();
            TIME = std::chrono::steady_clock::now();
            WRITE = true;
            // go to target
            handle.set_target_position(target);
            std::this_thread::sleep_for(std::chrono::milliseconds(OBSERVATION_TAU));
            // sleep until reached target
            while (std::abs(vars.get_current_position() - target) > 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(OBSERVATION_TAU));
            }

            // go back to home
            WRITE = false;
            write_positions_to_file(target); // save to csv
            handle.set_target_position(0);
            std::this_thread::sleep_for(std::chrono::milliseconds(OBSERVATION_TAU));
            // sleep until at home
            while (std::abs(vars.get_current_position() - target) > 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(OBSERVATION_TAU));
            }
        }
    }

    return 0;
}