#include "tic_lib.hpp"

tic::handle open_handle(const char *desired_serial_number)
{
    // Get a list of Tic devices connected via USB.
    std::vector<tic::device> list = tic::list_connected_devices();

    // Iterate through the list and select one device.
    for (const tic::device &device : list)
    {
        if (desired_serial_number &&
            device.get_serial_number() != desired_serial_number)
        {
            // Found a device with the wrong serial number, so continue on to
            // the next device in the list.
            continue;
        }
        std::cout << device.get_serial_number() << "\n";

        // Open a handle to this device and return it.
        return tic::handle(device);
    }

    throw std::runtime_error("No device found.");
}

uint64_t millis(std::chrono::_V2::system_clock::time_point time)
{
    uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      time.time_since_epoch())
                      .count();
    return ms;
}
