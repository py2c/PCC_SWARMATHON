#!/bin/bash

rosnode kill $HOSTNAME\_MOBILITY
rostopic pub -1 /$HOSTNAME\/mobility geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
rosnode kill $HOSTNAME\_ABRIDGE
rosnode kill $HOSTNAME\_NAVSAT
rosnode kill $HOSTNAME\_EKF
rosnode kill $HOSTNAME\_CAMERA
rosnode kill $HOSTNAME\_OBSTACLE
rosnode kill $HOSTNAME\_TARGET
rosnode kill ublox_gps

pkill camera
pkill mobility
pkill obstacle
pkill target
pkill abridge
pkill ublox_gps
pkill navsat_transfor
pkill ekf_localizatio
