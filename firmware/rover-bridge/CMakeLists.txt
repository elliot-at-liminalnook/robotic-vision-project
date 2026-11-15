# The rover's board: carries the servo bus over Wi-Fi and stops the wheels when the hub goes quiet.
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rover_bridge)
