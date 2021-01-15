cmake_minimum_required(VERSION 3.17)
project(statemachine)

set(CMAKE_CXX_STANDARD 20)

add_executable(statemachine main.cpp app.cpp states.cpp)