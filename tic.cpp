// Uses the Tic's C++ API to send and receive data from a Tic.
// NOTE: The Tic's control mode must be "Serial / I2C / USB".

#include <iostream>
#include <tic.hpp>

// Opens a handle to a Tic that can be used for communication.
//
// To open a handle to any Tic:
//   tic_handle * handle = open_handle();
// To open a handle to the Tic with serial number 01234567:
//   tic_handle * handle = open_handle("01234567");
tic::handle open_handle(const char *desired_serial_number = nullptr)
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

        // Open a handle to this device and return it.
        return tic::handle(device);
    }

    throw std::runtime_error("No device found.");
}

int main()
{
    try
    {
        tic::handle handle = open_handle();

        tic::variables vars = handle.get_variables();

        int32_t position = vars.get_current_position();
        std::cout << "Current position is " << position << ".\n";

        int32_t new_target = 75000;
        std::cout << "Setting target position to " << new_target << ".\n";

        handle.exit_safe_start();
        handle.set_target_position(new_target);
    }
    catch (const std::exception &error)
    {
        std::cerr << "Error: " << error.what() << std::endl;
        return 1;
    }
    return 0;
}