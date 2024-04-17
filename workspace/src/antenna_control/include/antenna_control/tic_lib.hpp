#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <tic.hpp>

namespace data_collection
{
    tic::variables TIC_VARS;
    tic::handle H(nullptr);
    const int OBSERVATION_TAU = 20;
    bool RUNNING = true;
    bool WRITE = false;
    std::chrono::_V2::steady_clock::time_point TIME;
    std::vector<std::pair<int64_t, int>> POSITIONS;
    std::mutex MUTEX;
    std::string FILENAME;
    void thread_get_position();

} // namespace data_collection

// Opens a handle to a Tic that can be used for communication.
tic::handle open_handle(const char *desired_serial_number = nullptr);
uint64_t millis(std::chrono::_V2::system_clock::time_point time);
