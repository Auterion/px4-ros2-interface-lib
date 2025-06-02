## Autonomous Acro Mode Example

This is an example mode that autonomously flies a power loop.

When activated, there are two steps:
1. Building up forward velocity, using velocity setpoints and the current vehicle's heading.
1. When the target velocity is reached, the power loop is entered.
  This uses a rate setpoint with a constant pitch rate and dynamic thrust.

Via RC Roll stick, the radius of the loop can be adjusted dynamically.
And the yaw rate can be adjusted via RC Yaw stick.

### Requirements

The multicopter must fulfill the following requirements:
- A well-tuned rate controller
- Enable `MC_AIRMODE` (including yaw)
- Set `MC_PITCHRATE_MAX` >= velocity / radius * 180 / pi (400 deg/s should be sufficient)
- Set the hover thrust in the code or via ROS parameter according to the vehicle's hover thrust.
  Ideally, the hover thrust is <= 0.25. A hover thrust of 0.3 will still work, but significantly above that leads to performance degradation.
  See below for details.
- If desired, increase `MC_YAWRATE_MAX` to the maximum yaw rate you want to use.
  Be aware that a higher yaw rate will require more thrust, and if too high, the vehicle will start to lose altitude.
- Increase the maximum velocity `MPC_XY_VEL_MAX` to e.g. 20 m/s.

#### Thrust feasibility
There are two requirements in order not to exceed the minimum or maximum thrust:
1. At the lower end of the loop:
  ```
  hover_thrust / G * ((velocity / radius) ^ 2) * radius < 1 - 2 * hover_thrust
  ```
1. At the upper end of the loop:
  ```
  1 / G * ((velocity / radius) ^ 2) * radius > 2
  ```
  For both to be feasible, the hover_thrust must be <= 0.25 (a slightly higher value will still work though).

Radius and velocity pairs that work well (`v = sqrt(2 * G * r)`):

| Radius [m] | Velocity [m/s] |
|------------|----------------|
| 1.0        | 4.0            |
| 2.0        | 6.0            |
| 3.0        | 7.0            |
| 5.0        | 10.0           |
| 8.0        | 12.5           |
| 10.0       | 14.0           |
| 15.0       | 17.0           |


The default gains for the controller should be fine.

### Implementation
The power loop is controlled using a constant pitch rate and dynamic thrust.
The thrust consists of three parts:
1. Centripetal force (constant)
2. Gravity compensation
3. A PD controller based on the position error w.r.t. the center of the loop

Note: Roll is not controlled, and thus the vehicle can drift sidewards with respect to the loop over time.
Similarly, absolute yaw is not controlled either.
