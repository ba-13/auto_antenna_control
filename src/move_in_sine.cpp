#include "auto_antenna_control/tic_lib.hpp"
#include <thread>
#include <math.h>
#include <chrono>

tic::handle handle;
tic::variables vars;

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

const int tau = 500;
double freq = 2;

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
        std::cout << value << " " << handle.get_variables().get_current_position() << "\n";
    }

    return 0;
}