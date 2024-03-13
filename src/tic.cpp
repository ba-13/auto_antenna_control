// Uses the Tic's C++ API to send and receive data from a Tic.
// NOTE: The Tic's control mode must be "Serial / I2C / USB".

#include "auto_antenna_control/tic_lib.hpp"

int main()
{
    try
    {
        tic::handle handle = open_handle();

        tic::variables vars = handle.get_variables();
        handle.energize();

        int32_t position = vars.get_current_position();

        std::cout << "Max speed is:" << vars.get_max_speed() << ".\n";
        std::cout << "Current position is " << position << ".\n";

        int32_t new_target = 0;
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