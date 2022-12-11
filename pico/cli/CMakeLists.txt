cmake_minimum_required(VERSION 3.13)

project(OpenThermOstat-CLI VERSION 0.1 LANGUAGES C CXX ASM)

find_package(CURL REQUIRED)
find_library(MOSQUITTO_LIB mosquitto REQUIRED)

add_executable(otcli otcli.cpp)
target_include_directories(otcli PRIVATE ${CURL_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
target_link_libraries(otcli PRIVATE ${CURL_LIBRARIES} ${MOSQUITTO_LIB} pthread)