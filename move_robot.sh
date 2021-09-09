#!/bin/bash

echo "Moving the robot back and forth in order to calibrate the localization.."

# drive backwards - publish cmd_vel with 10hz for 30s
timeout 30s rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear:  {x: -3.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

# drive straight ahead again to spawn
timeout 30s rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear:  {x: 3.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
