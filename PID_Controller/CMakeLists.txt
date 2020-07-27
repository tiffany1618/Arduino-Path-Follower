cmake_minimum_required(VERSION 3.16)
project(Robot)

set(CMAKE_CXX_STANDARD 14)

add_executable(PID_Controller src/PID_Controller.cpp src/PID_Controller.h)