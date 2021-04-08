# kangaroo_x2_driver

ROS driver for Kangaroo X2 motor driver board

Takes a clean copy of the Kangaroo Arduino library provided by Dimension Engineering, and provides a reimplementation of Arduino.h in order to connect to the X2 through a linux serial port instead of Arduino serial.

The intention is to use this to control the X2 from a Raspberry Pi through an FTDI USB-serial driver.

Kangaroo X2 Arduino library:
https://www.dimensionengineering.com/info/arduino

## Test the connection to the motor driver:

```
rosrun kangaroo_x2_driver SpeedControlExample
```

(this will run motor 1 back and forward)

## Create separate controllers for motors 1 and 2:

```
roslaunch kangaroo_x2_driver independent_speed_control.launch
```

The controllers listen for speed commands of type std_msgs/Float64 on topics:

```
/motor_1_controller/command
/motor_2_controller/command
```

## Differential drive control:

```
roslaunch kangaroo_x2_driver diff_drive_control.launch
```

The differential-drive controller will listen for twist velocity commands published to topic `/diff_drive_controller/cmd_vel`.

## Kangaroo Setup

Ensure the kangaroo board is configured for independent mode (dip-switch 4 is ON) and digital input (dip-switch 1 is ON). Also ensure the board has been tuned with these settings.
