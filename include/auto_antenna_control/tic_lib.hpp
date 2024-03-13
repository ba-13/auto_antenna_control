#include <iostream>
#include <tic.hpp>

// Opens a handle to a Tic that can be used for communication.
tic::handle open_handle(const char *desired_serial_number = nullptr);