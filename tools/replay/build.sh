#!/bin/sh
# Build the log replay tool (needs g++; Git Bash on Windows works). Output: tools/replay/replay(.exe)
cd "$(dirname "$0")" || exit 1
FW="../../Astro 331 Lab 8"
g++ -O2 -std=c++17 -I"$FW/include" replay.cpp "$FW/src/attitude_ekf.cpp" "$FW/src/mag_aid.cpp" -o replay
