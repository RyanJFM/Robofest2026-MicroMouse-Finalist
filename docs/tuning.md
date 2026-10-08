# Robot Tuning

Mansoyanno contains several tunable parameters for adapting the robot to different maze conditions and hardware behavior.

## Main Tuning Categories

### Wall Detection

The IR sensors use thresholds to determine whether a nearby wall is present.

### Cell Movement

Encoder ticks are used to estimate cell distance.

### Turning

Gyroscope-based turning parameters control 90° and 180° rotations.

### Arc Motion

The arc fast-run system includes parameters for:

* Turning radius
* Track width
* Arc fraction
* Early-turn distance
* Acceleration
* Deceleration
* Minimum corner speed

### Wireless Tuning

The HC-06 Bluetooth module provides a wireless serial interface for monitoring and tuning the robot.

Detailed values should be updated here as the final competition-tested configuration is documented.
