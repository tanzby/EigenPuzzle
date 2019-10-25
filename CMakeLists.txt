cmake_minimum_required(VERSION 3.5)
project(EigenPuzzle)

set(CMAKE_CXX_FLAGS "-std=c++17 -O")

add_executable(EigenPuzzle main.cpp Solver.h)