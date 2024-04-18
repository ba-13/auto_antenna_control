#include "antenna_control/tic_lib.hpp"

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
    // std::cout << device.get_serial_number() << "\n";

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

namespace data_collection
{

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
        auto since = std::chrono::duration_cast<std::chrono::milliseconds>(
            start_time - TIME);

        MUTEX.lock();
        // most prolly push_back when increasing size would
        // lead to sampling time deviation
        POSITIONS.push_back({since.count(), TIC_VARS.get_current_position()});
        MUTEX.unlock();
      }

      // sleep for remaining time
      auto end_time = std::chrono::steady_clock::now();
      auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(
          end_time - start_time);

      auto remaining_time =
          std::chrono::milliseconds(OBSERVATION_TAU) - elapsed_time;

      if (remaining_time.count() > 0)
      {
        std::this_thread::sleep_for(remaining_time);
      }
    }
  }

} // namespace data_collection