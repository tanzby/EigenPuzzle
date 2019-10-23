cmake_minimum_required(VERSION 3.5)
project(EigenPuzzle)

set(CMAKE_CXX_STANDARD 17)

add_executable(EigenPuzzle main.cpp Solver.h)
add_executable(ids ids.cpp)
add_executable(acm_style acm_style.cpp)