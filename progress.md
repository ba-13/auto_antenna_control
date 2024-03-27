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