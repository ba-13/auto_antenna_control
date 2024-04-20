# Progress tracking

## Day 1

We started by using the [NEMA Stepper Motors](https://robokits.co.in/motors/stepper-motor/stepper-motor-with-gearbox/nema17-planetary-geared-stepper-motor-14kgcm?products_id=2115:bdc4626aa1d1df8e14d80d345b2a442d), also [here](https://thinkrobotics.com/products/nema-17-planetary-gear-stepper-motor-100-1) with the [Tic 36v4](https://www.pololu.com/product/3141) motor controller.

Due to the fantastic documentation of the board, and the design of the stepper motor, we were able to perform position and velocity control, with a simple cmd [tool](https://www.pololu.com/docs/0J71/4.4). We also used the [code base](https://github.com/pololu/pololu-tic-software) for a handle for the tic controller, that made controlling the same with C++ as easy as it gets.

Some code that was useful:

```bash
sudo ldconfig # for syncing all libraries igq

ticcmd --list
ticcmd --status
ticcmd --energize
ticcmd --exit-safe-start --velocity 2000000
ticcmd --exit-safe-start --position 400
```

The problems to face:
- The stepper motor keeps track of it's position due to it's step nature, you've to take into account the recaliberating of the motors at every start.
- We already have the position and velocity control of individual motors at this stage, we need to take into consideration the extra torque faced by each angle control when the other is moving, leading to a disturbance in a true sense. We somehow need to reject that.
- Given the position control, we can create a pipeline that would translate those position to the next predicted angles and vice-versa.
- Once the predictor is created, we need to segment it somehow so that the path taken by the position controller is more nearer to our predicted trajectory instead of the control path chosen by the coupled motors.

## Day 2

Understood the apis provided by the motor-driver tic handle, including `halt_and_set_position`, `get_variables` and `energize`.
Realised the problem of knowing the actual position of the motor with respect to it's environment, because it starts at 0 step at every restart.

## Day 3

The step angle of the stepper motor NEMA17 is 1.8deg. But at full step, we observed the motor to complete full rotation at nearly 800 steps, which should have been 200 instead.

## Day 4

Received the Raspberry Pi 4 with packages pre-installed. Able to connect to it using personal hotspot. Connect your own laptop to the hotspot, find it's ipv4 alloted, and use that as subnet mask for the nmap scan

```bash
nmap -sn 192.168.31.0/24
```

This would give the connected hosts IPs, out of which one would be the `ubuntu` RPi. Connect to it using:

```bash
sshpass -p spinlab307 ssh ubuntu@192.168.31.217
```

## Day 5

Laid out the planning of the rest of the project

## Day 6

Refactored and converted the codebase to ROS workspace
Took response curves, analyze using:

```bash
python plot_positions.py --file ./data/u20/oneeigth/data4000000.csv
```

Some notes after taking responses to target positions:

- Irrespective of the step mode, same target position deviation takes the same amount of time.
- Taken data can be used to model the position control such that given target and current speed, one should be able to predict the curve taken. This would help to choose such waypoints that would have non-zero speeds by giving a furthur one as target.
- Need to obtain curves with non-zero initial speed (may be needed explicitly)

# Day 9

Azimuth - 19.2

# Day 10

Exporting libraries from other ROS packages, [here](https://jbohren.com/articles/modular-ros-packages#using-libraries-from-other-packages)

Refactoring led to increased delay! Have to backtrace to check where the problem lies in.

# Day 11

Antenna is now tracking a simulated UAV. The UAV pose predictions are a bit weird, and have to correct publishing of either just steps
or just degrees

# Day 12

An idea: Can include a feedback from motor control back to predictor about the average lag induced between the passing of angle target packets between them, this can lead to an improved time_horizon
