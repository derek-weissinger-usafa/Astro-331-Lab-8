#!/bin/sh
# Build the SLEW TEST / SLEW YAW host simulation (g++; Git Bash works). Run: ./sim [yaw|fixed] [room torque mN m]
cd "$(dirname "$0")" || exit 1
FW="../../Astro 331 Lab 8"
g++ -O2 -std=c++17 -I"$FW/include" sim.cpp "$FW/src/slew_test.cpp" "$FW/src/attitude_ctrl.cpp" "$FW/src/tune.cpp" -o sim
