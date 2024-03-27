#include "auto_antenna_control/tic_lib.hpp"
#include <thread>
#include <math.h>
#include <chrono>

tic::handle handle;
tic::variables vars;
const int obs_tau = 5;
const int tau = 100;
double freq = 2;
bool running = true;

double get_sine_freq(double freq, double time)
{
    return sin(2 * M_PI * freq * time);
}

uint64_t millis(std::chrono::_V2::system_clock::time_point time)
{
    uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      time.time_since_epoch())
                      .count();
    return ms;
}

void get_position()
{
    while (running)
    {
        vars = handle.get_variables();
        std::this_thread::sleep_for(std::chrono::milliseconds(obs_tau));
    }
}

int main()
{
    try
    {
        handle = open_handle();
    }
    catch (const std::exception &error)
    {
        std::cerr << "Error: " << error.what() << std::endl;
        return 1;
    }

    vars = handle.get_variables();

    handle.halt_and_set_position(0);
    handle.energize();

    std::thread t_position(get_position);

    auto start = std::chrono::high_resolution_clock::now();
    uint64_t start_millis = millis(start);
    auto curr = start;
    while (true)
    {
        curr = std::chrono::high_resolution_clock::now();
        uint64_t curr_millis = millis(curr);
        double value = get_sine_freq(freq, (curr_millis - start_millis) / 1000.0);
        value *= 1000;
        handle.set_target_position(floor(value));
        std::this_thread::sleep_for(std::chrono::milliseconds(tau));
        while (std::abs(vars.get_current_position() - value) > 10)
        {
            // std::cout << "waiting for getting to position\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(obs_tau));
        }
        std::cout << value << " " << vars.get_current_position() << "\n";
    }

    t_position.join();
    return 0;
}