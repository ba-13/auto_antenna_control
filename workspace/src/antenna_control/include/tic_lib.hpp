#include <iostream>
#include <chrono>
#include <tic.hpp>

// Opens a handle to a Tic that can be used for communication.
tic::handle open_handle(const char *desired_serial_number = nullptr);
uint64_t millis(std::chrono::_V2::system_clock::time_point time);