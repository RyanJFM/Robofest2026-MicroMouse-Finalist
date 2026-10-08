# Motion Control

Mansoyanno uses multiple feedback systems to control its movement.

## Encoders

Magnetic encoders on the N20 motors provide wheel movement feedback.

Encoder information is used to control cell-distance movement and maintain consistent motion.

## Gyroscope

The ICM-20602 provides angular-rate measurements.

The robot integrates the Z-axis gyro data to estimate its current heading angle.

Gyroscope feedback is used for controlled turns such as:

* 90° left
* 90° right
* 180° turns

## IR Wall Feedback

The side-facing IR sensors measure nearby walls.

These measurements can be used to correct the robot's position while travelling through a cell.

## Pivot Turns

The pivot fast-run mode rotates the robot around its approximate center.

This approach is slower than arc cornering but provides a predictable and repeatable cornering behavior.

## Arc Turns

The arc fast-run mode allows the robot to maintain forward motion while cornering.

This reduces the time lost during turns and allows a substantially faster run.

The trade-off is that arc motion requires more careful tuning of:

* Track width
* Turning radius
* Arc fraction
* Wheel speed
* Acceleration
* Deceleration
* Wall alignment
