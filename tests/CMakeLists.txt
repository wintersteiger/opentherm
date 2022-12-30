cmake_minimum_required(VERSION 3.13)
project(OpenThermTests VERSION 0.1 LANGUAGES C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(pid_tests pid_tests.cpp)
target_include_directories(pid_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)

add_test(NAME pid_tests COMMAND pid_tests)